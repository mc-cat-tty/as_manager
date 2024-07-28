#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace as::logstream {
  enum class OutputLog {
    STANDARD = 0,
    ROS
  };

template <OutputLog typeOfOutput = OutputLog::STANDARD>
class Logstream : public std::ostream {
public:
    Logstream(std::string_view level, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_publisher = nullptr) : std::ostream(&buf), level(level), ros_publisher(ros_publisher) {}

    template <typename T>
    Logstream& operator<<(const T& data) {
        std::ostringstream oss;
        oss << "[" << level << "] " << data;
        buf.sputn(oss.str().c_str(), oss.str().size());
        return *this;
    }

    Logstream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
            if constexpr (typeOfOutput == OutputLog::STANDARD) {
                std::cout << std::endl;
            } else if (ros_publisher!=nullptr) {
                std_msgs::msg::String msg;
                msg.data = "\n";
                ros_publisher->publish(msg);
            }
        }
        return *this;
    }

private:
    class Buffer : public std::streambuf {
    public:
        int overflow(int c) override {
            if (c != EOF) {
                if constexpr (typeOfOutput == OutputLog::STANDARD) {
                    std::cout.put(c);
                } else {
                    std_msgs::msg::String msg;
                    msg.data = std::string(c);
                    ros_publisher->publish(msg);
                }
            }
            return c;
        }
    };

    Buffer buf;
    std::string_view level;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_publisher;
};
  auto log_node= rclcpp::Node::make_shared("log_publisher_node");
  auto log_publisher = log_node->create_publisher<std_msgs::msg::String>("log_topic", 10);
  auto error_node= rclcpp::Node::make_shared("log_publisher_node");
  auto error_publisher = error_node->create_publisher<std_msgs::msg::String>("error_topic", 10);
  Logstream log=Logstream<OutputLog::ROS>("LOG",log_publisher);
  Logstream error=Logstream<OutputLog::ROS>("ERRORE",error_publisher);
}
