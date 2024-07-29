#include "as_manager.hpp"

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>


void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Killing as_manager node" << std::endl;
    rclcpp::shutdown();
  }
}


int main(int argc, char *argv[]) {
  signal(SIGINT, handleSignal);
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<AsManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
