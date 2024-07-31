#include <as_manager/as_manager.hpp>

using namespace std::chrono_literals;

ROSInputState AsManagerNode::inputState;
ROSPublishers AsManagerNode::outputPublishers;
ROSSubscribers AsManagerNode::inputSubscribers;

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
  signalUpdater(signals::utils::Updater<5>::getInstance()),
  superloopTimer(this->create_wall_timer(
    50ms,
    std::bind(&AsManagerNode::superloop, this)
  )) {

    this->load_parameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

    AsManagerNode::outputPublishers.asStatePublisher = this->create_publisher<std_msgs::msg::String>(this->asStateTopic, 1);
    AsManagerNode::outputPublishers.brakePercentagePublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>(this->brakeTopic, 1);
    AsManagerNode::outputPublishers.gearPublisher = this->create_publisher<can_msgs::msg::Frame>(this->canSendTopic, 1);
    AsManagerNode::outputPublishers.clutchPublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>(this->clutchTopic, 1);

    using namespace std::placeholders;
    AsManagerNode::inputSubscribers.rpm_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      this->ecuTopic, 10, std::bind(&AsManagerNode::ecuRpmCb, this, _1));
    AsManagerNode::inputSubscribers.brakePressureFront_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
       this->ecuTopic, 10, std::bind(&AsManagerNode::brakePressureFrontCb, this, _1));
    AsManagerNode::inputSubscribers.brakePressureRear_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
       this->ecuTopic, 10, std::bind(&AsManagerNode::brakePressureRearCb, this, _1));
    AsManagerNode::inputSubscribers.EBS1Pressure_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
       this->ecuTopic, 10, std::bind(&AsManagerNode::ebs1PressureCb, this, _1));
    AsManagerNode::inputSubscribers.EBS2Pressure_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
       this->ecuTopic, 10, std::bind(&AsManagerNode::ebs2PressureCb, this, _1));
    AsManagerNode::inputSubscribers.resStatus_subscriber = this->create_subscription<mmr_kria_base::msg::ResStatus>(
      this->resTopic, 10, std::bind(&AsManagerNode::resStatusCb, this, _1));
    AsManagerNode::inputSubscribers.maxonMotors_subscriber = this->create_subscription<mmr_kria_base::msg::ActuatorStatus>(
      this->maxonStateTopic, 10, std::bind(&AsManagerNode::maxonMotorCb, this, _1));
    AsManagerNode::inputSubscribers.ASMission_subscriber = this->create_subscription<can_msgs::msg::Frame>(
      this->canReceiveTopic, 10, std::bind(&AsManagerNode::asMissionCb, this, _1));
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
