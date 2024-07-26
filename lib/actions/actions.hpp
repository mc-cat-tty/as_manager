#pragma once

#include <hal/hal.hpp>

namespace hal::actions {
    void open_sdc();
    void close_sdc();

    void brake_act1();
    void brake_act2();
    void brake_all_act();

    void unbrake_act1();
    void unbrake_act2();
    void unbrake_all_act();

    void active_buzzer();
    void disabled_buzzer();

    void switch_off_assi_Y();
    void switch_on_assi_Y();
    void strobe_assi_Y();

    void switch_off_assi_B();
    void switch_on_assi_B();
    void strobe_assi_B();
}