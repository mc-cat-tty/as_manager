#ifdef TEST

#include <as_manager/hal/hal.hpp>
#include <as_manager/as_manager.hpp>

namespace hal{
  bool read_orin_on() { return false; }
  bool read_can_open_on() { return AsManagerNode::getCanOpenOn(); }
  float read_ebs1_pressure() { return 6.0; }
  float read_ebs2_pressure() { return 6.0; }
  float read_brake_pressure_front() { return 50.0; }
  float read_brake_pressure_rear() { return 50.0; }
  SdcState read_sdc() { return SdcState::CLOSE; }
  unsigned read_rpm() {return AsManagerNode::getEngineRpm(); }
  ResState read_res_state() { return ResState::OPERATIONAL; }
  uint8_t read_res_bit_vector() {return AsManagerNode::getResState();}
  bool read_asms_status() { return true; }
  bool read_stop_message()  {return AsManagerNode::getStopMessage();}
  bool read_mission_status() { return false; }
  uint8_t read_motors_bit_vector() { return 0x07; }
  bool is_autonomous_mission() { return AsManagerNode::isAutonomousMission(); }

  void toggle_watchdog_state(bool pinState) { std::cerr << "wdg: " << pinState << std::endl; }
  void toggle_actuator1_state(ActuatorState state) { std::cerr << "act1 toggled" << std::endl; }
  void toggle_actuator2_state(ActuatorState state) { std::cerr << "act1 toggled" << std::endl; }
  void toggle_sdc_state(SdcState state) { std::cerr << "sdc toggled" << std::endl; }
  void set_assi_Y_state(KriaPin::Value pinState) { std::cerr << "assiY toggled" << std::endl; }
  void set_assi_B_state(KriaPin::Value pinState) { std::cerr << "assiB toggled" << std::endl; }
  void set_buzzer_state(KriaPin::Value pinState) { std::cerr << "buzzer toggled" << std::endl; }
  void write_watchdog_state(KriaPin::Value pinState) { std::cerr << "wdg: " << (bool) pinState << std::endl; }
  void send_brake_pressure_percentage(float percentage) { AsManagerNode::sendBrakePercentage(percentage); }
  void send_current_state(as::AsState state) { AsManagerNode::sendASState(state); }
  void pull_clutch() { std::cerr << "Clutch pulled" << std::endl; }
  void enable_motors() { AsManagerNode::enableMotors(true, true); }
  void set_gear(uint8_t gear) { std::cerr << "Shift to gear: " << gear << std::endl; }
}

#endif