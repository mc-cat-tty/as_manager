#include "as_manager.hpp"


AsManagerNode::AsManagerNode() : Node("as_manager_node") {
  auto& watchdog = watchdog::Watchdog::getInstance();
  auto& assi_manager = as::assi_manager::AssiManager::getInstance();
  auto EbsSupervisor = as::ebs_supervisor::EbsSupervisor();
  auto& updater = signals::utils::Updater<5>::getInstance();
  
}

AsManagerNode::~AsManagerNode() {
  RCLCPP_INFO(this->get_logger(), "Destroying AS Manager...");
  rclcpp::shutdown();
}


/*


  while(1) {
    updater.update();
    EbsSupervisor.run();
    watchdog.run();
    assi_manager.run();
  }
}
*/