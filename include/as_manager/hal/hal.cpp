#ifndef TEST

#include <as_manager/hal/hal.hpp>
#include <as_manager/as_manager.hpp>
#include <as_manager/hal/pin_implementation.hpp>

namespace hal {
  SdcState read_sdc() {return pin::sdcSensPin.getValue() == KriaPin::Value::ON ? SdcState::OPEN : SdcState::CLOSE;}
  bool read_asms_status() {return pin::asmsPin.getValue() == KriaPin::Value::ON ? false : true ;}

  void toggle_actuator1_state(ActuatorState state) {
    pin::ebs1Pin.setValue(state == ActuatorState::BRAKING ? KriaPin::Value::OFF : KriaPin::Value::ON);
  }
  void toggle_actuator2_state(ActuatorState state) {
    pin::ebs2Pin.setValue(state == ActuatorState::BRAKING ? KriaPin::Value::OFF : KriaPin::Value::ON);
  }

  void write_watchdog_state(KriaPin::Value pinState) {
    pin::watchdogPin.setValue(pinState);
  }
  void set_assi_Y_state(KriaPin::Value pinState) {
    pin::assiyPin.setValue(pinState);
  }
  void set_assi_B_state(KriaPin::Value pinState) {
    pin::assibPin.setValue(pinState);
  }
  void set_buzzer_state(KriaPin::Value pinState) {
    pin::buzzerPin.setValue(pinState);
  }
  void toggle_sdc_state(SdcState state) {
    pin::sdcCtrlPin.setValue(state == SdcState::OPEN ? KriaPin::Value::OFF : KriaPin::Value::ON);
  }

  uint8_t read_res_bit_vector() {return AsManagerNode::getResState();}
  ResState read_res_state() {return ResState::OPERATIONAL;}
  float read_brake_pressure_front()  { return AsManagerNode::getBrakePressureFront(); }
  float read_brake_pressure_rear()  {return AsManagerNode::getBrakePressureRear(); }
  unsigned read_rpm() {return AsManagerNode::getEngineRpm(); }
  float read_ebs1_pressure()  {return AsManagerNode::getEbsPressure1(); }
  float read_ebs2_pressure() {return AsManagerNode::getEbsPressure2(); }
  bool read_stop_message()  {return AsManagerNode::getStopMessage();}
  bool is_autonomous_mission()  {return AsManagerNode::getAutonomousMission(); }
  bool read_orin_on() { return AsManagerNode::getOrinOn(); }
  bool read_can_open_on() { return AsManagerNode::getCanOpenOn(); } 

  uint8_t read_motors_bit_vector() { return AsManagerNode::getMaxonMotorsState(); }

  void send_brake_pressure_percentage(float percentage) {
    AsManagerNode::sendBrakePercentage(percentage);
  }

  void send_current_state(as::AsState state) {
    AsManagerNode::sendASState(state);
  }

  void set_gear(uint8_t gear) {
    if (gear == 1) AsManagerNode::sendGearUp();
  }
  void pull_clutch() {
    AsManagerNode::sendClutchAction(true);
  }

  void enable_motors(){
    AsManagerNode::enableMotors(true,true);
  }
}

#endif  // TEST