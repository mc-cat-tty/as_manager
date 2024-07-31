#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>

#include <can_msgs/msg/frame.hpp>
#include <mmr_edf/edf_node.hpp>
#include <mmr_kria_base/msg/ecu_status.hpp>
#include <mmr_kria_base/msg/res_status.hpp>
#include <mmr_kria_base/msg/actuator_status.hpp>
#include <mmr_kria_base/msg/cmd_motor.hpp>


constexpr unsigned updatableSignalsNumber = 5;


struct ROSInputState {
  uint8_t resState, maxonMotorsState;
  unsigned engineRpm;
  float brakePressureFront, brakePressureRear;
  float ebsPressure1, ebsPressure2;
  bool stopMessage, autonomousMission;
};

struct ROSPublishers {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asStatePublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr brakePercentagePublisher;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr gearPublisher;
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr clutchPublisher;
};

struct ROSSubscribers {
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr rpm_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr brakePressureRear_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr brakePressureFront_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr EBS1Pressure_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::EcuStatus>::SharedPtr EBS2Pressure_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::ActuatorStatus>::SharedPtr maxonMotors_subscriber;
  rclcpp::Subscription<mmr_kria_base::msg::ResStatus>::SharedPtr resStatus_subscriber;
  //stop messagge da implementare
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr ASMission_subscriber;

};

class AsManagerNode : public EDFNode {
  private:
  as::ebs_supervisor::EbsSupervisor &ebsSupervisor;
  watchdog::Watchdog &watchdog;
  as::assi_manager::AssiManager &assiManager;
  signals::utils::Updater<updatableSignalsNumber> &signalUpdater;
  rclcpp::TimerBase::SharedPtr superloopTimer;

  //parameters
  std::string  brakeTopic, asStateTopic, canSendTopic, clutchTopic, ecuTopic, resTopic, maxonStateTopic, canReceiveTopic;
  bool debug;
  static int ebsTankPressureThreshold,brakePressureOneActuatorThreshold,brakePressureBothActuatorsThreshold,brakePressureMaxonMotorsThreshold, unbrakePressureThreshold;
  static float asmsAplha,sdcAplha,brakePressureFrontAlpha,brakePressureRearAlpha,rpmAlpha;

  static ROSInputState inputState;
  static ROSSubscribers inputSubscribers;
  static ROSPublishers outputPublishers;

  void superloop();

  void ecuRpmCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) { inputState.engineRpm = msg->nmot; }
  void brakePressureRearCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) { inputState.brakePressureRear = msg->p_brake_rear; }
  void brakePressureFrontCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) { inputState.brakePressureFront = msg->p_brake_front; }
  void ebs1PressureCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) { inputState.ebsPressure1 = msg->p_ebs_1; }
  void ebs2PressureCb(const mmr_kria_base::msg::EcuStatus::SharedPtr msg) { inputState.ebsPressure2 = msg->p_ebs_2; }
  void resStatusCb(const mmr_kria_base::msg::ResStatus::SharedPtr msg){ inputState.resState = (msg->emergency << 2) | (msg->bag << 1) | msg->go_signal; }
  void maxonMotorCb(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg){ inputState.maxonMotorsState = (msg->brake_status << 2) | (msg->steer_status << 1) | msg->clutch_status; }
  void asMissionCb(const can_msgs::msg::Frame::SharedPtr msg) { if (msg->id == 0x40) inputState.autonomousMission = msg->data[0]; }
  // void stopMessageCb(const mmr_kria_base::msg::ActuatorStatus::SharedPtr msg) {}

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
  static inline bool getAutonomousMission() { return inputState.autonomousMission; }

  // Output state setters
  static inline void sendASState(as::EbsSupervisorState state) {
    auto msg = std_msgs::msg::String();
    msg.data = as::EbsSupervisorStateLookup.at(state);
    outputPublishers.asStatePublisher->publish(msg);
  }

  static inline void sendBrakePercentage(float percentage){
    auto msg = mmr_kria_base::msg::CmdMotor();
    msg.brake_torque = percentage;
    outputPublishers.brakePercentagePublisher->publish(msg);
  }

  static inline void sendGear(uint8_t gear){ // TODO: 
    auto msg = can_msgs::msg::Frame();
    msg.id = 0x610;
    msg.data[0] = gear;
    outputPublishers.gearPublisher->publish(msg);
  }

  static inline void sendClutchAction(bool doPull){
    auto msg = mmr_kria_base::msg::CmdMotor();
    msg.clutch_disengage = doPull;
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
    declare_parameter("topic.ecuTopic", "");
    declare_parameter("topic.resTopic", "");
    declare_parameter("topic.maxonStateTopic", "");
    declare_parameter("topic.canReceiveTopic", "");

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
    get_parameter("topic.ecuTopic", this->ecuTopic);
    get_parameter("topic.resTopic", this->resTopic);
    get_parameter("topic.maxonStateTopic", this->maxonStateTopic);
    get_parameter("topic.canReceiveTopic", this->canReceiveTopic);

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