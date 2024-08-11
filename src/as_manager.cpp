#include <as_manager/as_manager.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

ROSInputState AsManagerNode::inputState;
ROSSubscribers AsManagerNode::inputSubscriptions;
ROSPublishers AsManagerNode::outputPublishers;


AsManagerNode::AsManagerNode() :
  EDFNode("as_manager_node")
   {

    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

    watchdog = &watchdog::Watchdog::getInstance();
    assiManager = &as::assi_manager::AssiManager::getInstance();
    ebsSupervisor = &as::ebs_supervisor::EbsSupervisor::getInstance();
    signalUpdater = &signals::utils::Updater<5>::getInstance();

    auto &transientLocalQOS = rclcpp::QoS(1)
      .reliable()
      .keep_last(1)
      .transient_local();

    AsManagerNode::outputPublishers.asStatePublisher = this->create_publisher<std_msgs::msg::UInt8>(this->asStateTopic, transientLocalQOS);
    AsManagerNode::outputPublishers.brakePublisher = this->create_publisher<mmr_base::msg::CmdMotor>(this->brakeTopic, 1);
    AsManagerNode::outputPublishers.gearPublisher = this->create_publisher<mmr_base::msg::CmdEcu>(this->gearUpTopic, 1);
    AsManagerNode::outputPublishers.clutchPublisher = this->create_publisher<mmr_base::msg::CmdMotor>(this->clutchTopic, 1);
    AsManagerNode::outputPublishers.steerPublisher = this->create_publisher<mmr_base::msg::CmdMotor>(this->steerTopic, 1);

    using namespace std::placeholders;

    const auto &bestEffortQOS = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.ecuStatusSubscription= this->create_subscription<mmr_base::msg::EcuStatus>(
      this->ecuStatusTopic, bestEffortQOS, std::bind(&AsManagerNode::ecuStatusCb, this, _1)
    );
    
    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.resStatusSubscription= this->create_subscription<mmr_base::msg::ResStatus>(
      this->resStatusTopic, bestEffortQOS, std::bind(&AsManagerNode::resStatusCb, this, _1)
    );
    
    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.maxonMotorsSubscription= this->create_subscription<mmr_base::msg::ActuatorStatus>(
      this->maxonMotorsTopic, bestEffortQOS, std::bind(&AsManagerNode::maxonMotorsCb, this, _1)
    );
    
    // Reliable 1 keep last
    AsManagerNode::inputSubscriptions.missionSelectedSubscription= this->create_subscription<std_msgs::msg::Int8>(
      this->missionSelectedTopic, 1, std::bind(&AsManagerNode::missionSelectedCb, this, _1)
    );

    // Reliable 1 keep last
    AsManagerNode::inputSubscriptions.stopMessageSubscription= this->create_subscription<std_msgs::msg::Bool>(
      this->stopMessageTopic, 1, std::bind(&AsManagerNode::stopMessageCb, this, _1)
    );
  }

AsManagerNode::~AsManagerNode() {
  RCLCPP_INFO(this->get_logger(), "Destroying AS Manager...");
  rclcpp::shutdown();
}

void AsManagerNode::superloop() {
  this->signalUpdater->update();
  this->ebsSupervisor->run();
  this->watchdog->run();
  this->assiManager->run();
}

void AsManagerNode::loadParameters(){
  declare_parameter("generic.WCET", 5000000);
  declare_parameter("generic.period", 10000000);
  declare_parameter("generic.deadline", 10000000);
  declare_parameter("generic.debug", false);

  declare_parameter("topics.asStateTopic", "");
  declare_parameter("topics.brakeTopic", "");
  declare_parameter("topics.gearUpTopic", "");
  declare_parameter("topics.clutchTopic", "");
  declare_parameter("topics.steerTopic", "");
  declare_parameter("topics.ecuStatusTopic", "");
  declare_parameter("topics.resStatusTopic", "");
  declare_parameter("topics.maxonMotorsTopic", "");
  declare_parameter("topics.missionSelectedTopic", "");
  declare_parameter("topics.stopMessageTopic", "");

  declare_parameter("thresholds.ebsTankPressureThreshold", 5);
  declare_parameter("thresholds.brakePressureOneActuatorThreshold", 20);
  declare_parameter("thresholds.brakePressureBothActuatorsThreshold", 10);
  declare_parameter("thresholds.brakePressureMaxonMotorsThreshold", 6);
  declare_parameter("thresholds.unbrakePressureThreshold", 5);

  declare_parameter("alpha.asmsAlpha", 0.8f);
  declare_parameter("alpha.sdcAlpha",  0.8f);
  declare_parameter("alpha.brakePressureFrontAlpha",  0.8f);
  declare_parameter("alpha.brakePressureRearAlpha",  0.8f);
  declare_parameter("alpha.rpmAlpha",  0.8f);

  get_parameter("generic.WCET", this->m_nWCET);
  get_parameter("generic.period", this->m_nPeriod);
  get_parameter("generic.deadline", this->m_nDeadline);

  get_parameter("topics.asStateTopic", this->asStateTopic);
  get_parameter("topics.brakeTopic", this->brakeTopic);
  get_parameter("topics.gearUpTopic", this->gearUpTopic);
  get_parameter("topics.clutchTopic", this->clutchTopic);
  get_parameter("topics.steerTopic", this->steerTopic);
  get_parameter("topics.ecuStatusTopic", this->ecuStatusTopic);
  get_parameter("topics.resStatusTopic", this->resStatusTopic);
  get_parameter("topics.maxonMotorsTopic", this->maxonMotorsTopic);
  get_parameter("topics.missionSelectedTopic", this->missionSelectedTopic);
  get_parameter("topics.stopMessageTopic", this->stopMessageTopic);

  using namespace params;
  get_parameter("generic.debug", Parameters::getInstance().debug);

  get_parameter("thresholds.ebsTankPressureThreshold", Parameters::getInstance().ebsTankPressureThreshold);
  get_parameter("thresholds.brakePressureOneActuatorThreshold", Parameters::getInstance().brakePressureOneActuatorThreshold);
  get_parameter("thresholds.brakePressureBothActuatorsThreshold", Parameters::getInstance().brakePressureBothActuatorsThreshold);
  get_parameter("thresholds.brakePressureMaxonMotorsThreshold", Parameters::getInstance().brakePressureMaxonMotorsThreshold);
  get_parameter("thresholds.unbrakePressureThreshold", Parameters::getInstance().unbrakePressureThreshold);

  get_parameter("alpha.asmsAlpha", Parameters::getInstance().asmsAlpha);
  get_parameter("alpha.sdcAlpha", Parameters::getInstance().sdcAlpha);
  get_parameter("alpha.brakePressureFrontAlpha", Parameters::getInstance().brakePressureFrontAlpha);
  get_parameter("alpha.brakePressureRearAlpha", Parameters::getInstance().brakePressureRearAlpha);
  get_parameter("alpha.rpmAlpha", Parameters::getInstance().rpmAlpha);
  
}
