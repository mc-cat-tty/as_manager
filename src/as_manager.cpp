#include <as_manager/as_manager.hpp>

using namespace std::chrono_literals;

ROSInputState AsManagerNode::inputState;
ROSPublishers AsManagerNode::outputPublishers;
ROSSubscribers AsManagerNode::inputSubscribers;

AsManagerNode::AsManagerNode() :
  Node("as_manager_node"),
  ebsSupervisor(as::ebs_supervisor::EbsSupervisor::getInstance()),
  watchdog(watchdog::Watchdog::getInstance()),
  assiManager(as::assi_manager::AssiManager::getInstance()),
  signalUpdater(signals::utils::Updater<5>::getInstance()),
  superloopTimer(this->create_wall_timer(
    50ms,
    std::bind(&AsManagerNode::superloop, this)
  )) {
    AsManagerNode::outputPublishers.asStatePublisher = this->create_publisher<std_msgs::msg::String>("/as/status", 1);
    // AsManagerNode::outputPublishers.brakePercentagePublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>("/command/brake", 1);
    // AsManagerNode::outputPublishers.gearPublisher = this->create_publisher<can_msg::msg::Frame>("/canbus/rx/msg", 1);
    // AsManagerNode::outputPublishers.clutchPublisher = this->create_publisher<mmr_kria_base::msg::CmdMotor>("/command/clutch", 1);

    using namespace std::placeholders;
    AsManagerNode::inputSubscribers.rpm_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      "status/ecu", 10, std::bind(&AsManagerNode::ecuRpmCb, this, _1));
    AsManagerNode::inputSubscribers.brakePressureFront_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      "status/ecu", 10, std::bind(&AsManagerNode::brakePressureFrontCb, this, _1));
    AsManagerNode::inputSubscribers.brakePressureRear_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      "status/ecu", 10, std::bind(&AsManagerNode::brakePressureRearCb, this, _1));
    AsManagerNode::inputSubscribers.EBS1Pressure_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      "status/ecu", 10, std::bind(&AsManagerNode::ebs1PressureCb, this, _1));
    AsManagerNode::inputSubscribers.EBS2Pressure_subscriber = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
      "status/ecu", 10, std::bind(&AsManagerNode::ebs2PressureCb, this, _1));
    AsManagerNode::inputSubscribers.resStatus_subscriber = this->create_subscription<mmr_kria_base::msg::ResStatus>(
      "status/res", 10, std::bind(&AsManagerNode::resStatusCb, this, _1));
    AsManagerNode::inputSubscribers.maxonMotors_subscriber = this->create_subscription<mmr_kria_base::msg::ActuatorStatus>(
      "actuator/status", 10, std::bind(&AsManagerNode::maxonMotorCb, this, _1));
    AsManagerNode::inputSubscribers.ASMission_subscriber = this->create_subscription<can_msgs::msg::Frame>(
      "canbus/tx/msg", 10, std::bind(&AsManagerNode::asMissionCb, this, _1));
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
