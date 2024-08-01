#include <as_manager/as_manager.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

ROSInputState AsManagerNode::inputState;
ROSSubscribers AsManagerNode::inputSubscriptions;
ROSPublishers AsManagerNode::outputPublishers;

int AsManagerNode::ebsTankPressureThreshold;
int AsManagerNode::brakePressureOneActuatorThreshold;
int AsManagerNode::brakePressureBothActuatorsThreshold;
int AsManagerNode::brakePressureMaxonMotorsThreshold;
int AsManagerNode::unbrakePressureThreshold;
float AsManagerNode::asmsAplha;
float AsManagerNode::sdcAplha;
float AsManagerNode::brakePressureFrontAlpha;
float AsManagerNode::brakePressureRearAlpha;
float AsManagerNode::rpmAlpha;

AsManagerNode::AsManagerNode() :
  EDFNode("as_manager_node"),
  ebsSupervisor(as::ebs_supervisor::EbsSupervisor::getInstance()),
  watchdog(watchdog::Watchdog::getInstance()),
  assiManager(as::assi_manager::AssiManager::getInstance()),
  signalUpdater(signals::utils::Updater<5>::getInstance()) {

    this->load_parameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

    auto &transientLocalQOS = rclcpp::QoS(1)
      .reliable()
      .keep_last(1)
      .transient_local();

    AsManagerNode::outputPublishers.asStatePublisher = this->create_publisher<std_msgs::msg::UInt8>(this->asStateTopic, transientLocalQOS);
    AsManagerNode::outputPublishers.brakePublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>(this->brakeTopic, 1);
    AsManagerNode::outputPublishers.gearPublisher = this->create_publisher<can_msgs::msg::Frame>(this->canSendTopic, 1);
    AsManagerNode::outputPublishers.clutchPublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>(this->clutchTopic, 1);
    AsManagerNode::outputPublishers.steerPublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>(this->steerTopic, 1);

    using namespace std::placeholders;

    const auto &bestEffortQOS = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.ecuStatusSubscription= this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      this->ecuStatusTopic, bestEffortQOS, std::bind(&AsManagerNode::ecuStatusCb, this, _1)
    );
    
    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.resStatusSubscription= this->create_subscription<mmr_kria_base::msg::ResStatus>(
      this->resStatusTopic, bestEffortQOS, std::bind(&AsManagerNode::resStatusCb, this, _1)
    );
    
    // Best effort 1 keep last
    AsManagerNode::inputSubscriptions.maxonMotorsSubscription= this->create_subscription<mmr_kria_base::msg::ActuatorStatus>(
      this->maxonMotorsTopic, bestEffortQOS, std::bind(&AsManagerNode::maxonMotorsCb, this, _1)
    );
    
    // Reliable 1 keep last
    AsManagerNode::inputSubscriptions.missionSelectedSubscription= this->create_subscription<std_msgs::msg::String>(
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
  this->signalUpdater.update();
  this->ebsSupervisor.run();
  this->watchdog.run();
  this->assiManager.run();
}
