#include <as_manager/as_manager.hpp>

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
