#pragma once
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/hal/utils.hpp>
#include <as_manager/signals/updater.hpp>

#include <can_msgs/msg/frame.hpp>
#include <mmr_edf/edf_node.hpp>
#include <mmr_kria_base/msg/ecu_status.hpp>
#include <mmr_kria_base/msg/res_status.hpp>
#include <mmr_kria_base/msg/actuator_status.hpp>
#include <mmr_kria_base/msg/cmd_motor.hpp>

constexpr unsigned updatableSignalsNumber = 5;
using namespace std::chrono_literals;

struct ROSInputState {
  uint8_t resState, maxonMotorsState;
  unsigned engineRpm;
  float brakePressureFront, brakePressureRear;
  float ebsPressure1, ebsPressure2;
  bool stopMessage;
  std::string autonomousMission;
};

struct ROSPublishers {
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr asStatePublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr brakePercentagePublisher;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr gearPublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr clutchPublisher;
};

struct ROSSubscribers {
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr ecuStatusSubscription;
  rclcpp::Subscription<mmr_kria_base::msg::ResStatus>::SharedPtr resStatusSubscription;
  rclcpp::Subscription<mmr_kria_base::msg::ActuatorStatus>::SharedPtr maxonMotorsSubscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr missionSelectedSubscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopMessageSubscription;
};

class AsManagerNode : public EDFNode {
  private:
  as::ebs_supervisor::EbsSupervisor &ebsSupervisor;
  watchdog::Watchdog &watchdog;
  as::assi_manager::AssiManager &assiManager;
  signals::utils::Updater<updatableSignalsNumber> &signalUpdater;
  rclcpp::TimerBase::SharedPtr superloopTimer;

  // Parameters
  std::string brakeTopic, asStateTopic, canSendTopic, clutchTopic, ecuStatusTopic, resStatusTopic, maxonMotorsTopic, missionSelectedTopic, stopMessageTopic;  
  bool debug;
  static int ebsTankPressureThreshold, brakePressureOneActuatorThreshold, brakePressureBothActuatorsThreshold, brakePressureMaxonMotorsThreshold, unbrakePressureThreshold;
  static float asmsAplha,sdcAplha,brakePressureFrontAlpha,brakePressureRearAlpha,rpmAlpha;

  // Static state
  static ROSInputState inputState;
  static ROSSubscribers inputSubscriptions;
  static ROSPublishers outputPublishers;

  void superloop();

  // Callbacks
  void ecuStatusCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) {
    inputState.engineRpm = msg->nmot;
    inputState.brakePressureRear = msg->p_brake_rear;
    inputState.brakePressureFront = msg->p_brake_front;
    inputState.ebsPressure1 = msg->p_ebs_1;
    inputState.ebsPressure2 = msg->p_ebs_2;
  }

  void resStatusCb(const mmr_kria_base::msg::ResStatus::SharedPtr msg) {
    inputState.resState = hal::utils::resComposeBv(msg->go_signal, msg->bag, msg->emergency);
  }
  
  void maxonMotorsCb(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg) {
    inputState.maxonMotorsState = hal::utils::motorsComposeBv(msg->clutch_status, msg->steer_status, msg->brake_status);
  }
  
  void missionSelectedCb(const std_msgs::msg::String::SharedPtr mission) {
      std::transform(
        std::begin(mission->data),
        std::end(mission->data),
        std::begin(inputState.autonomousMission),
        [](char c){ return std::tolower(c); }
    ) ;
  }
  
  void stopMessageCb(const std_msgs::msg::Bool::SharedPtr msg) {
    inputState.stopMessage = msg->data;
  }

  public:
  AsManagerNode();
  ~AsManagerNode();

  // Getter of parameters
  static inline int getEbsTankPressureThreshold() { return ebsTankPressureThreshold; }
  static inline int getBrakePressureOneActuatorThreshold() { return brakePressureOneActuatorThreshold; }
  static inline int getBrakePressureBothActuatorsThreshold() { return brakePressureBothActuatorsThreshold; }
  static inline int getBrakePressureMaxonMotorsThreshold() { return brakePressureMaxonMotorsThreshold; }
  static inline int getUnbrakePressureThreshold() { return unbrakePressureThreshold; }
  static inline float getAsmsAlpha() { return asmsAplha; }
  static inline float getSdcAlpha() { return sdcAplha; }
  static inline float getBrakePressureFrontAlpha() { return brakePressureFrontAlpha; }
  static inline float getBrakePressureRearAlpha() { return brakePressureRearAlpha; }
  static inline float getRpmAlpha() { return rpmAlpha; }

  // Input state getters
  static inline uint8_t getResState() { return inputState.resState; }
  static inline uint8_t getMaxonMotorsState() { return inputState.maxonMotorsState; }
  static inline unsigned getEngineRpm() { return inputState.engineRpm; }
  static inline float getBrakePressureFront() { return inputState.brakePressureFront; }
  static inline float getBrakePressureRear() { return inputState.brakePressureRear; }
  static inline float getEbsPressure1() { return inputState.ebsPressure1; }
  static inline float getEbsPressure2() { return inputState.ebsPressure2; }
  static inline bool getStopMessage() { return inputState.stopMessage; }
  static inline bool getAutonomousMission() { return inputState.autonomousMission != "manual"; }

  // Output state setters
  static inline void sendASState(as::AsState state) {
    auto msg = std_msgs::msg::UInt8();
    msg.data = static_cast<uint8_t>(state);
    outputPublishers.asStatePublisher->publish(msg);
  }

  static inline void sendBrakePercentage(float percentage){
    auto msg = mmr_kria_base::msg::CmdMotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePercentagePublisher->publish(msg);
  }

  static inline void sendGearUp(){
    auto msg = can_msgs::msg::Frame();
    msg.id = 0x610;

    auto sendFun = [&msg](bool val) {
      msg.data[0] = val << 1;
      outputPublishers.gearPublisher->publish(msg);
    };

    hal::utils::ecuButtonTrigger(sendFun, 1ms);
  }

  static inline void sendClutchAction(bool doDisengage) {
    auto msg = mmr_kria_base::msg::CmdMotor();
    msg.disengaged = doDisengage;
    outputPublishers.clutchPublisher->publish(msg);
  }

  void load_parameters(){
    declare_parameter("generic.WCET", 5000000);
    declare_parameter("generic.period", 10000000);
    declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.debug", false);

    declare_parameter("topic.asStateTopic", "");
    declare_parameter("topic.brakeTopic", "");
    declare_parameter("topic.canSendTopic", "");
    declare_parameter("topic.clutchTopic", "");
    declare_parameter("topic.ecuStatusTopic", "");
    declare_parameter("topic.resStatusTopic", "");
    declare_parameter("topic.maxonMotorsTopic", "");
    declare_parameter("topic.missionSelectedTopic", "");
    declare_parameter("topic.stopMessageTopic", "");

    declare_parameter("thresholds.ebsTankPressureThreshold", 5);
    declare_parameter("thresholds.brakePressureOneActuatorThreshold", 20);
    declare_parameter("thresholds.brakePressureBothActuatorsThreshold", 10);
    declare_parameter("thresholds.brakePressureMaxonMotorsThreshold", 6);
    declare_parameter("thresholds.unbrakePressureThreshold", 5);

    declare_parameter("alpha.asmsAplha", 0.8f);
    declare_parameter("alpha.sdcAplha",  0.8f);
    declare_parameter("alpha.brakePressureFrontAlpha",  0.8f);
    declare_parameter("alpha.brakePressureRearAlpha",  0.8f);
    declare_parameter("alpha.rpmAlpha",  0.8f);

    get_parameter("generic.WCET", this->m_nWCET);
    get_parameter("generic.period", this->m_nPeriod);
    get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.debug", this->debug);

    get_parameter("topic.asStateTopic", this->asStateTopic);
    get_parameter("topic.brakeTopic", this->brakeTopic);
    get_parameter("topic.canSendTopic", this->canSendTopic);
    get_parameter("topic.clutchTopic", this->clutchTopic);
    get_parameter("topic.ecuStatusTopic", this->ecuStatusTopic);
    get_parameter("topic.resStatusTopic", this->resStatusTopic);
    get_parameter("topic.maxonMotorsTopic", this->maxonMotorsTopic);
    get_parameter("topic.missionSelectedTopic", this->missionSelectedTopic);
    get_parameter("topic.stopMessageTopic", this->stopMessageTopic);

    get_parameter("thresholds.ebsTankPressureThreshold", this->ebsTankPressureThreshold);
    get_parameter("thresholds.brakePressureOneActuatorThreshold", this->brakePressureOneActuatorThreshold);
    get_parameter("thresholds.brakePressureBothActuatorsThreshold", this->brakePressureBothActuatorsThreshold);
    get_parameter("thresholds.brakePressureMaxonMotorsThreshold", this->brakePressureMaxonMotorsThreshold);
    get_parameter("thresholds.unbrakePressureThreshold", this->unbrakePressureThreshold);

    get_parameter("alpha.asmsAplha", this->asmsAplha);
    get_parameter("alpha.sdcAplha", this->sdcAplha);
    get_parameter("alpha.brakePressureFrontAlpha", this->brakePressureFrontAlpha);
    get_parameter("alpha.brakePressureRearAlpha", this->brakePressureRearAlpha);
    get_parameter("alpha.rpmAlpha", this->rpmAlpha);
    
  }
};