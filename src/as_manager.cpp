#include <as_manager/as_manager.hpp>

using namespace std::chrono_literals;

ROSInputState AsManagerNode::inputState;
ROSOutputState AsManagerNode::outputPublishers;

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
