#pragma once

#include <cstdint>
#include <random>
#include <chrono>
#include <common/common_type.hpp>

namespace hal {
    using namespace as;
  
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
    int read_rpm();
    float read_ebs1_pressure() ;
    float read_ebs2_pressure() ;
    bool read_stop_message() ;
    uint8_t read_motors_bit_vector();

    void send_brake_pressure_percentage(float percentage);
    void send_current_state(EbsSupervisorState state);

    double random_number();
}