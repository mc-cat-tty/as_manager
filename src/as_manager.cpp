#include "as_manager.hpp"

using namespace std::chrono_literals;

AsManagerNode::AsManagerNode() :
  Node("as_manager_node"),
  ebsSupervisor(as::ebs_supervisor::EbsSupervisor()),
  watchdog(watchdog::Watchdog::getInstance()),
  assiManager(as::assi_manager::AssiManager::getInstance()),
  signalUpdater(signals::utils::Updater<5>::getInstance())
  {
  
}

AsManagerNode::~AsManagerNode() {
  RCLCPP_INFO(this->get_logger(), "Destroying AS Manager...");
  rclcpp::shutdown();
}

void AsManagerNode::superLoop() {
  this->signalUpdater.update();
  this->ebsSupervisor.run();
  this->watchdog.run();
  this->assiManager.run();
}
