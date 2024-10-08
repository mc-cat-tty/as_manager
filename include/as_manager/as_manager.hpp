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
  bool stopMessage;
  int8_t autonomousMission;
};

struct ROSPublishers {
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr asStatePublisher;
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
};

class AsManagerNode : public EDFNode {
  private:
  as::ebs_supervisor::EbsSupervisor *ebsSupervisor;
  watchdog::Watchdog *watchdog;
  as::assi_manager::AssiManager *assiManager;
  signals::utils::Updater<updatableSignalsNumber> *signalUpdater;

  // Static state
  static ROSInputState inputState;
  static ROSSubscribers inputSubscriptions;
  static ROSPublishers outputPublishers;

  // Topics
  std::string brakeTopic, asStateTopic, gearUpTopic, clutchTopic, steerTopic, ecuStatusTopic, resStatusTopic, maxonMotorsTopic, missionSelectedTopic, stopMessageTopic;

  // Callbacks
  void ecuStatusCb(const mmr_base::msg::EcuStatus::SharedPtr msg) {
    inputState.engineRpm = msg->nmot;
    inputState.brakePressureRear = msg->p_brake_rear;
    inputState.brakePressureFront = msg->p_brake_front;
    inputState.ebsPressure1 = msg->p_ebs_1;
    inputState.ebsPressure2 = msg->p_ebs_2;
  }

  void resStatusCb(const mmr_base::msg::ResStatus::SharedPtr msg) {
    inputState.resState = hal::utils::resComposeBv(msg->go_signal, msg->bag, msg->emergency);
  }
  
  void maxonMotorsCb(const mmr_base::msg::ActuatorStatus::SharedPtr msg) {
    inputState.maxonMotorsState = hal::utils::motorsComposeBv(msg->clutch_status, msg->steer_status, msg->brake_status);
  }
  
  void missionSelectedCb(const std_msgs::msg::Int8::SharedPtr mission) {
    inputState.autonomousMission = mission->data;
  }
  
  void stopMessageCb(const std_msgs::msg::Bool::SharedPtr msg) {
    inputState.stopMessage = msg->data;
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
  static inline bool getAutonomousMission() { return inputState.autonomousMission != COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_MANUAL; }

  // Output state setters
  static inline void sendASState(as::AsState state) {
    auto msg = std_msgs::msg::UInt8();
    msg.data = static_cast<uint8_t>(state);
    outputPublishers.asStatePublisher->publish(msg);
  }

  static inline void sendBrakePercentage(float percentage){
    auto msg = mmr_base::msg::CmdMotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePublisher->publish(msg);
  }

  static inline void setUpMotors(bool brake, bool clutch, bool steer){
    if(brake){
      auto msgBrake=mmr_base::msg::CmdMotor();
      msgBrake.enable=brake;
      outputPublishers.brakePublisher->publish(msgBrake);
    }

    if(clutch){
      auto msgClutch=mmr_base::msg::CmdMotor();
      msgClutch.enable=clutch;
      outputPublishers.brakePublisher->publish(msgClutch);
    } 

    if(steer){
      auto msgSteerHoming=mmr_base::msg::CmdMotor();
      msgSteerHoming.homing=steer;
      outputPublishers.brakePublisher->publish(msgSteerHoming);
      
      auto msgSteerEnable=mmr_base::msg::CmdMotor();
      msgSteerEnable.enable=steer;
      outputPublishers.brakePublisher->publish(msgSteerEnable);
    }
  }

  static inline void sendGearUp(){
    auto msg = mmr_base::msg::CmdEcu();
    msg.gear_target = 1;
    outputPublishers.gearPublisher->publish(msg);
  }

  static inline void sendClutchAction(bool doDisengage) {
    auto msg = mmr_base::msg::CmdMotor();
    msg.disengaged = doDisengage;
    outputPublishers.clutchPublisher->publish(msg);
  }

  void load_parameters(){
    declare_parameter("generic.WCET", 5000000);
    declare_parameter("generic.period", 10000000);
    declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.debug", false);

    declare_parameter("topics.asStateTopic", "");
    declare_parameter("topics.brakeTopic", "");
    declare_parameter("topics.gearUpTopic", "");
    declare_parameter("topics.clutchTopic", "");
    declare_parameter("topics.steerTopic", "");
    declare_parameter("topics.ecuStatusTopic", "");
    declare_parameter("topics.resStatusTopic", "");
    declare_parameter("topics.maxonMotorsTopic", "");
    declare_parameter("topics.missionSelectedTopic", "");
    declare_parameter("topics.stopMessageTopic", "");

    declare_parameter("thresholds.ebsTankPressureThreshold", 5);
    declare_parameter("thresholds.brakePressureOneActuatorThreshold", 20);
    declare_parameter("thresholds.brakePressureBothActuatorsThreshold", 10);
    declare_parameter("thresholds.brakePressureMaxonMotorsThreshold", 6);
    declare_parameter("thresholds.unbrakePressureThreshold", 5);

    declare_parameter("alpha.asmsAlpha", 0.8f);
    declare_parameter("alpha.sdcAlpha",  0.8f);
    declare_parameter("alpha.brakePressureFrontAlpha",  0.8f);
    declare_parameter("alpha.brakePressureRearAlpha",  0.8f);
    declare_parameter("alpha.rpmAlpha",  0.8f);

    get_parameter("generic.WCET", this->m_nWCET);
    get_parameter("generic.period", this->m_nPeriod);
    get_parameter("generic.deadline", this->m_nDeadline);

    get_parameter("topics.asStateTopic", this->asStateTopic);
    get_parameter("topics.brakeTopic", this->brakeTopic);
    get_parameter("topics.gearUpTopic", this->gearUpTopic);
    get_parameter("topics.clutchTopic", this->clutchTopic);
    get_parameter("topics.steerTopic", this->steerTopic);
    get_parameter("topics.ecuStatusTopic", this->ecuStatusTopic);
    get_parameter("topics.resStatusTopic", this->resStatusTopic);
    get_parameter("topics.maxonMotorsTopic", this->maxonMotorsTopic);
    get_parameter("topics.missionSelectedTopic", this->missionSelectedTopic);
    get_parameter("topics.stopMessageTopic", this->stopMessageTopic);

    using namespace params;
    get_parameter("generic.debug", Parameters::getInstance().debug);

    get_parameter("thresholds.ebsTankPressureThreshold", Parameters::getInstance().ebsTankPressureThreshold);
    get_parameter("thresholds.brakePressureOneActuatorThreshold", Parameters::getInstance().brakePressureOneActuatorThreshold);
    get_parameter("thresholds.brakePressureBothActuatorsThreshold", Parameters::getInstance().brakePressureBothActuatorsThreshold);
    get_parameter("thresholds.brakePressureMaxonMotorsThreshold", Parameters::getInstance().brakePressureMaxonMotorsThreshold);
    get_parameter("thresholds.unbrakePressureThreshold", Parameters::getInstance().unbrakePressureThreshold);

    get_parameter("alpha.asmsAlpha", Parameters::getInstance().asmsAlpha);
    get_parameter("alpha.sdcAlpha", Parameters::getInstance().sdcAlpha);
    get_parameter("alpha.brakePressureFrontAlpha", Parameters::getInstance().brakePressureFrontAlpha);
    get_parameter("alpha.brakePressureRearAlpha", Parameters::getInstance().brakePressureRearAlpha);
    get_parameter("alpha.rpmAlpha", Parameters::getInstance().rpmAlpha);
    
  }
};