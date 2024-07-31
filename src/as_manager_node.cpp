#include <as_manager/as_manager.hpp>

void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Killing as_manager node" << std::endl;
    rclcpp::shutdown();
  }
}

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  signal(SIGINT, handleSignal);
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    decltype(auto) node = std::make_shared<AsManagerNode>();
    executor.add_node(node);

    while (true) {
      executor.spin_all(50ms);
      node->superloop();
      sched_yield();
    }

    rclcpp::shutdown();
  }
  catch (const rclcpp::exceptions::InvalidNodeError &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
