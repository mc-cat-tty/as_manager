#pragma once

#include <cstdint>
#include <random>
#include <chrono>
#include <unistd.h>
#include <as_manager/common/common_types.hpp>

namespace hal {
  SdcState read_sdc();
  bool read_asms_status() ;

  void toggle_actuator1_state(ActuatorState state) ;
  void toggle_actuator2_state(ActuatorState state) ;
  void write_watchdog_state(bool pinState);
  void set_assi_Y_state(AssiState state);
  void set_assi_B_state(AssiState state);
  void set_buzzer_state(BuzzerState state);
  void toggle_sdc_state(SdcState state);

  uint8_t read_res_bit_vector();
  ResState read_res_state();
  float read_brake_pressure_front() ;
  float read_brake_pressure_rear() ;
  unsigned read_rpm();
  float read_ebs1_pressure() ;
  float read_ebs2_pressure() ;
  bool read_stop_message() ;
  bool read_go_message();
  bool is_autonomous_mission();
  bool read_orin_on();
  uint8_t read_motors_bit_vector();

  void send_brake_pressure_percentage(float percentage);
  void send_current_state(as::AsState state);

  void set_gear(uint8_t gear);
  void pull_clutch();
  void enable_motors();
}