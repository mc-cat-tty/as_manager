#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>

constexpr unsigned updatableSignalsNumber = 5;

struct ROSInputState {
  uint8_t resState, maxonMotorsState;
  unsigned engineRpm;
  float brakePressureFront, brakePressureRear;
  float ebsPressure1, ebsPressure2;
  bool stopMessage, autonomousMission;
};

struct ROSOutputState {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asStatePublisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brakePercentagePublisher;
  rclcpp::Publisher<std_msgs::msg::Int>::SharedPtr gearPublisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clutchPublisher;
};

class AsManagerNode : public rclcpp::Node {
  private:
  as::ebs_supervisor::EbsSupervisor &ebsSupervisor;
  watchdog::Watchdog &watchdog;
  as::assi_manager::AssiManager &assiManager;
  signals::utils::Updater<updatableSignalsNumber> &signalUpdater;
  rclcpp::TimerBase::SharedPtr superloopTimer;

  static ROSInputState inputState;
  static ROSOutputState outputPublishers;

  void superloop();

  public:
  AsManagerNode();
  ~AsManagerNode();

  // Input state getters
  static inline uint8_t getResState() { return inputState.resState; }
  static inline uint8_t getMaxonMotorsState() { return inputState.maxonMotorsState; }
  static inline unsigned getEngineRpm() { return inputState.engineRpm; }
  static inline float getBrakePressureFront() { return inputState.brakePressureFront; }
  static inline float getBrakePressureRear() { return inputState.brakePressureRear; }
  static inline float getEbsPressure1() { return inputState.ebsPressure1; }
  static inline float getEbsPressure2() { return inputState.ebsPressure2; }
  static inline bool getStopMessage() { return inputState.stopMessage; }
  static inline bool getAutonomousMission() { return inputState.autonomousMission; }

  // Output state setters
  static inline void sendASState(std::string_view state) {
    auto msg = std_msgs::msg::String();
    msg.data = state;
    outputPublishers.asStatePublisher->publish(msg);
  }

  static inline void sendBrakePercentage(float percentage){
    auto mmr_kria_base::msg::cmdmotor msg = mmr_kria_base::msg::cmdmotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePercentagePublisher->publish(msg);
  }

  static inline void sendGear(uint8_t gear){ // TODO: 
    auto can_msg::msg::Frame msg = mmr_kria_base::msg::cmdmotor();
    msg.
    outputPublishers.gearPublisher->publish(msg);
  }

  static inline void sendClutchPull(int percentage){
    auto mmr_kria_base::msg::cmdmotor msg = mmr_kria_base::msg::cmdmotor();
    msg.disengaged = 1;
    outputPublishers.clutchPublisher->publish(msg);
  }
};