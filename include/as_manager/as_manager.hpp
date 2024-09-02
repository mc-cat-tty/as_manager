#pragma once
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/hal/utils.hpp>
#include <as_manager/signals/updater.hpp>
#include <as_manager/params/parameters.hpp>

#include <can_msgs/msg/frame.hpp>
#include <mmr_edf/mmr_edf.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/res_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/msg/cmd_motor.hpp>
#include <mmr_base/msg/cmd_ecu.hpp>
#include <mmr_base/configuration.hpp>

constexpr unsigned updatableSignalsNumber = 5;
using namespace std::chrono_literals;

struct ROSInputState {
  uint8_t resState, maxonMotorsState;
  unsigned engineRpm;
  float brakePressureFront, brakePressureRear;
  float ebsPressure1, ebsPressure2;
  bool stopMessage = false;
  bool orinOn = false;
  bool canOpenOn = false;
  int8_t mission = static_cast<int8_t>(COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_IDLE);
};

struct ROSPublishers {
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr asStatePublisher;
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr brakePublisher;
  rclcpp::Publisher<mmr_base::msg::CmdEcu>::SharedPtr gearPublisher;
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr clutchPublisher;
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr steerPublisher;
};

struct ROSSubscribers {
  rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr ecuStatusSubscription;
  rclcpp::Subscription<mmr_base::msg::ResStatus>::SharedPtr resStatusSubscription;
  rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr maxonMotorsSubscription;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr missionSelectedSubscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopMessageSubscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orinOnSubscription;
};

class AsManagerNode : public EDFNode {
  private:
  as::ebs_supervisor::EbsSupervisor *ebsSupervisor;
  watchdog::Watchdog *watchdog;
  as::assi_manager::AssiManager *assiManager;
  signals::utils::Updater<updatableSignalsNumber> *signalUpdater;
  
  void loadParameters(); 

  // Static state
  static ROSInputState inputState;
  static ROSSubscribers inputSubscriptions;
  static ROSPublishers outputPublishers;

  // Topics
  std::string brakeTopic, asStateTopic, gearUpTopic, clutchTopic, steerTopic, ecuStatusTopic, resStatusTopic, maxonMotorsTopic, missionSelectedTopic, stopMessageTopic, orinOnTopic;

  // Callbacks
  inline void logInputState() const {
    if (not params::Parameters::getInstance().verboseCallbacks) return;
    RCLCPP_INFO(this->get_logger(), "[resOn | MaxonMotors | engineRpm | brakePressureFront | brakePressureRear | ebsPressure1 | ebsPressure2 | stopMessage | orinOn | canOpenOn | mission]");
    RCLCPP_INFO(
      this->get_logger(),
      "[  %d  |     %x   |    %d   |         %.1f        |        %.1f        |      %.1f     |      %.1f     |      %d      |    %d   |     %d     |    %d   ]",
      inputState.resState,
      inputState.maxonMotorsState,
      inputState.engineRpm,
      inputState.brakePressureFront,
      inputState.brakePressureRear,
      inputState.ebsPressure1,
      inputState.ebsPressure2,
      inputState.stopMessage,
      inputState.orinOn,
      inputState.canOpenOn,
      inputState.mission
    );
  }

  void ecuStatusCb(const mmr_base::msg::EcuStatus::SharedPtr msg) {
    inputState.engineRpm = msg->nmot;
    inputState.brakePressureRear = msg->p_brake_rear;
    inputState.brakePressureFront = msg->p_brake_front;
    inputState.ebsPressure1 = msg->p_ebs_1;
    inputState.ebsPressure2 = msg->p_ebs_2;
    logInputState();
  }

  void resStatusCb(const mmr_base::msg::ResStatus::SharedPtr msg) {
    inputState.resState = hal::utils::resComposeBv(msg->go_signal, msg->bag, msg->emergency);
    logInputState();
  }
  
  void maxonMotorsCb(const mmr_base::msg::ActuatorStatus::SharedPtr msg) {
    inputState.canOpenOn=true;
    inputState.maxonMotorsState = hal::utils::motorsComposeBv(msg->clutch_status, msg->steer_status, msg->brake_status);
    logInputState();
  }
  
  void missionSelectedCb(const std_msgs::msg::Int8::SharedPtr mission) {
    inputState.mission = mission->data;
    logInputState();
  }
  
  void stopMessageCb(const std_msgs::msg::Bool::SharedPtr msg) {
    inputState.stopMessage = msg->data;
    logInputState();
  }

  void orinOnCb(const std_msgs::msg::Bool::SharedPtr msg) {
    inputState.orinOn = msg->data;
    logInputState();
  }

  public:
  AsManagerNode();
  ~AsManagerNode();
  void superloop();

  // Input state getters
  static inline uint8_t getResState() { return inputState.resState; }
  static inline uint8_t getMaxonMotorsState() { return inputState.maxonMotorsState; }
  static inline unsigned getEngineRpm() { return inputState.engineRpm; }
  static inline float getBrakePressureFront() { return inputState.brakePressureFront; }
  static inline float getBrakePressureRear() { return inputState.brakePressureRear; }
  static inline float getEbsPressure1() { return inputState.ebsPressure1; }
  static inline float getEbsPressure2() { return inputState.ebsPressure2; }
  static inline bool getStopMessage() { return inputState.stopMessage; }
  static inline int8_t getMission() { return inputState.mission; }
  static inline bool getOrinOn() { return inputState.orinOn; }
  static inline bool getCanOpenOn() { return inputState.canOpenOn; }
  static inline bool isAutonomousMission() {
    auto mis = static_cast<COCKPIT::MMR_MISSION_VALUE>(inputState.mission);
    return mis != COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_MANUAL &&
           mis != COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_IDLE;
  }

  // Output state setters
  static inline void sendASState(as::AsState state) {
    auto msg = std_msgs::msg::Int8();
    msg.data = static_cast<int8_t>(state);
    outputPublishers.asStatePublisher->publish(msg);
  }

  static inline void sendBrakePercentage(float percentage){
    auto msg = mmr_base::msg::CmdMotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePublisher->publish(msg);
  }

  static inline void enableMotors(bool brake, bool clutch){
      auto msgBrake=mmr_base::msg::CmdMotor();
      msgBrake.enable=brake;
      outputPublishers.brakePublisher->publish(msgBrake);

      auto msgClutch=mmr_base::msg::CmdMotor();
      msgClutch.enable=clutch;
      outputPublishers.clutchPublisher->publish(msgClutch);
    
  }

  static inline void sendFirstGear(){
    auto msg = mmr_base::msg::CmdEcu();
    msg.gear_target = 1;
    outputPublishers.gearPublisher->publish(msg);
  }

  static inline void sendClutchAction(bool doDisengage) {
    auto msg = mmr_base::msg::CmdMotor();
    msg.disengaged = doDisengage;
    outputPublishers.clutchPublisher->publish(msg);
  }
};