#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>

constexpr unsigned updatableSignalsNumber = 5;

struct MaxonMotorState {
  uint8_t steer;
  uint8_t brake;
  uint8_t clutch;
}

struct ResStatus{
  uint8_t go;
  uint8_t emergency;
  uint8_t bag;
}

struct ROSInputState {
  ResStatus resState;
  MaxonMotorState maxonMotorsState;
  unsigned engineRpm;
  float brakePressureFront, brakePressureRear;
  float ebsPressure1, ebsPressure2;
  bool stopMessage, autonomousMission;
};

struct ROSPublisher {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asStatePublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr brakePercentagePublisher;
  rclcpp::Publisher<can_msg::msg::Frame>::SharedPtr gearPublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr clutchPublisher;
};

struct ROSSubscriber {
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr rpm_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr brakePressureRear_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr brakePressureFront_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr EBS1Pressure_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr EBS2Pressure_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::ActuatorStatus>::SharedPtr maxonMotors_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::ResStatus>::SharedPtr resStatus_subscriber;
  //stop messagge da implementare
  rclcpp::Subscription<can_msg::msg::Frame>::SharedPtr ASMission_subscriber;

};

class AsManagerNode : public rclcpp::Node {
  private:
  as::ebs_supervisor::EbsSupervisor &ebsSupervisor;
  watchdog::Watchdog &watchdog;
  as::assi_manager::AssiManager &assiManager;
  signals::utils::Updater<updatableSignalsNumber> &signalUpdater;
  rclcpp::TimerBase::SharedPtr superloopTimer;

  static ROSInputState inputState;
  static ROSSubscriber inputSubscribers;
  static ROSPublisher outputPublishers;

  void superloop();

   void rpm_callback(const mmr_kria_base::msg::EcuStatus::SharedPtr msg){
    inputState.rpm = msg->nmot;
  }

   void brakePressureRear_callback(const mmr_kria_base::msg::EcuStatus::SharedPtr msg){
    inputState.brakePressureRear = msg->p_brake_rear;
  }

   void brakePressureFront_callback(const mmr_kria_base::msg::EcuStatus::SharedPtr msg){
    inputState.brakePressureFront = msg->p_brake_front;
  }

   void ebs1Pressure_callback(const mmr_kria_base::msg::EcuStatus::SharedPtr msg){
    inputState.ebsPressure1 msg->p_ebs_1;
  }

   void ebs2Pressure_callback(const mmr_kria_base::msg::EcuStatus::SharedPtr msg){
    inputState.ebsPressure2 = msg->p_ebs_2;
  }

   void maxonMotor_callback(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg){
    inputState.maxonMotorsState.steer = msg->steer_status;
    inputState.maxonMotorsState.clutch msg->clutch_status;
    inputState.maxonMotorsState.brake = msg->brake_status;
  }

   void resStatus_callback(const mmr_kria_base::msg::ResStatus::SharedPtr msg){
    inputState.resState.go = msg->go;
    inputState.resState.emergency = msg->emergency;
    inputState.resState.bag = msg->bag;
  }

  static void asMission_callback(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg){
    //TODO
  }

  /*  static void stopMessage_callback(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg){
    //TODO
  }
  */



  public:
  AsManagerNode();
  ~AsManagerNode();

  // Input state getters
  static inline uint8_t getResState() {
    return (inputState.resState.Emergency << 2) | (inputState.resState.bag << 1) | inputState.resState.go;
  }
  static inline uint8_t getMaxonMotorsState() { 
    return (inputState.maxonMotorsState.brake << 2) | (inputState.maxonMotorsState.steer << 1) | inputState.maxonMotorsState.clutch;
  }
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
    auto mmr_kria_base::msg::CmdMotor msg = mmr_kria_base::msg::cmdmotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePercentagePublisher->publish(msg);
  }

  static inline void sendGear(uint8_t gear){ // TODO: 
    auto can_msg::msg::Frame msg = mmr_kria_base::msg::cmdmotor();
    msg.
    outputPublishers.gearPublisher->publish(msg);
  }

  static inline void sendClutchPull(int percentage){
    auto mmr_kria_base::msg::CmdMotor msg = mmr_kria_base::msg::cmdmotor();
    msg.disengaged = 1;
    outputPublishers.clutchPublisher->publish(msg);
  }
};