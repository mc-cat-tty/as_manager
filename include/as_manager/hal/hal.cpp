#ifndef TEST

#include <as_manager/hal/hal.hpp>
#include <as_manager/as_manager.hpp>

namespace hal {
  SdcState read_sdc() {return SdcState::Open;}
  bool read_asms_status() {return false;}

  void toggle_actuator1_state(ActuatorState state) {}
  void toggle_actuator2_state(ActuatorState state) {}
  void write_watchdog_state(bool pinState) {}
  void set_assi_Y_state(AssiState state) {}
  void set_assi_B_state(AssiState state) {}
  void set_buzzer_state(BuzzerState state) {}
  void toggle_sdc_state(SdcState state) {}

  uint8_t read_res_bit_vector() {return AsManagerNode::getResState();}
  ResState read_res_state() {return ResState::Operational;}
  float read_brake_pressure_front()  { return AsManagerNode::getBrakePressureFront(); }
  float read_brake_pressure_rear()  {return AsManagerNode::getBrakePressureRear(); }
  unsigned read_rpm() {return AsManagerNode::getEngineRpm}
  float read_ebs1_pressure()  {return AsManagerNode::getEbsPressure1(); }
  float read_ebs2_pressure() {return AsManagerNode::getEbsPressure2(); }
  bool read_stop_message()  {return false;}
  bool read_go_message() {return false;}
  bool is_autonomous_mission()  {return AsManagerNode::getAutonomousMission(); }

  uint8_t read_motors_bit_vector() {return 1;}

  void send_brake_pressure_percentage(float percentage) {
    AsManagerNode::sendBrakePressurePercentage(percentage);
  }

  void send_current_state(as::EbsSupervisorState state) {
    AsManagerNode::sendASState(as::EbsSupervisorStateLookup.at(state));
  }

  void set_gear(uint8_t gear) {
    AsManagerNode::sendGear(gear);
  }
  void pull_clutch() {
    AsManagerNode::sendClutch();
  }
}

#endif  // TEST