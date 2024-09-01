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

    AsManagerNode::outputPublishers.asStatePublisher = this->create_publisher<std_msgs::msg::Int8>(this->asStateTopic, transientLocalQOS);
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

    // Reliable 1 keep last
    AsManagerNode::inputSubscriptions.orinOnSubscription= this->create_subscription<std_msgs::msg::Bool>(
      this->orinOnTopic, transientLocalQOS, std::bind(&AsManagerNode::orinOnCb, this, _1)
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

void AsManagerNode::loadParameters() {
  using namespace params;
  decltype(auto) params = Parameters::getInstance();

  auto schedulerParamsProxy = ParametersProxy("scheduler", this);
  auto debugParamsProxy = ParametersProxy("debug", this);
  auto topicsParamsProxy = ParametersProxy("topics", this);
  auto thresholdsParamsProxy = ParametersProxy("ebsSupervisor.thresholds", this);
  auto filteringParamsProxy = ParametersProxy("ebsSupervisor.filtering", this);

  params = {
    .safetyFeatures = debugParamsProxy.get("safetyFeatures", true),
    .verboseCallbacks = debugParamsProxy.get("verboseCallbacks", false),
    .verboseHalReads = debugParamsProxy.get("verboseHalReads", false),

    .ebsTankPressureThreshold = thresholdsParamsProxy.get<int>("ebsTankPressureThreshold"),
    .brakePressureOneActuatorThreshold = thresholdsParamsProxy.get<int>("brakePressureOneActuatorThreshold"), 
    .brakePressureBothActuatorsThreshold = thresholdsParamsProxy.get<int>("brakePressureBothActuatorsThreshold"),
    .brakePressureMaxonMotorsThreshold = thresholdsParamsProxy.get<int>("brakePressureMaxonMotorsThreshold"),
    .unbrakePressureThreshold = thresholdsParamsProxy.get<int>("unbrakePressureThreshold"),
    
    .brakePressureFrontAlpha = filteringParamsProxy.get<float>("brakePressureFrontAlpha"),
    .brakePressureRearAlpha = filteringParamsProxy.get<float>("brakePressureRearAlpha"),
    .rpmAlpha = filteringParamsProxy.get<float>("rpmAlpha")
  };

  this->m_nWCET = schedulerParamsProxy.get("WCET", 5000000);
  this->m_nPeriod = schedulerParamsProxy.get("period", 10000000);
  this->m_nDeadline = schedulerParamsProxy.get("deadline", 10000000);

  this->asStateTopic = topicsParamsProxy.get<std::string>("asStateTopic");
  this->gearUpTopic = topicsParamsProxy.get<std::string>("gearUpTopic");
  this->brakeTopic = topicsParamsProxy.get<std::string>("brakeTopic");
  this->clutchTopic = topicsParamsProxy.get<std::string>("clutchTopic");
  this->steerTopic = topicsParamsProxy.get<std::string>("steerTopic");
  this->ecuStatusTopic = topicsParamsProxy.get<std::string>("ecuStatusTopic");
  this->resStatusTopic = topicsParamsProxy.get<std::string>("resStatusTopic");
  this->maxonMotorsTopic = topicsParamsProxy.get<std::string>("maxonMotorsTopic");
  this->missionSelectedTopic = topicsParamsProxy.get<std::string>("missionSelectedTopic");
  this->stopMessageTopic = topicsParamsProxy.get<std::string>("stopMessageTopic");
  this->orinOnTopic = topicsParamsProxy.get<std::string>("orinOnTopic");
}
