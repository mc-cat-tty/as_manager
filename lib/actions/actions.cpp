#include <actions/actions.hpp>

namespace hal::action {
  void open_sdc(){
    hal::toggle_sdc_state(SdcState::Open);
  }
  void close_sdc(){
    hal::toggle_sdc_state(SdcState::Closed);
  }

  void brake_act1(){
    hal::toggle_actuator1_state(ActuatorState::Braking);
  }

  void brake_act2(){
    hal::toggle_actuator2_state(ActuatorState::Braking);
  }

  void brake_all_act(){
    hal::toggle_actuator1_state(ActuatorState::Braking);
    hal::toggle_actuator2_state(ActuatorState::Braking);
  }

  void unbrake_act1(){
    hal::toggle_actuator1_state(ActuatorState::Unbrake);
  }
  void unbrake_act2(){
    hal::toggle_actuator2_state(ActuatorState::Unbrake);
  }
  void unbrake_all_act(){
    hal::toggle_actuator1_state(ActuatorState::Unbrake);
    hal::toggle_actuator2_state(ActuatorState::Unbrake);
  }

  void active_buzzer(){
    hal::set_buzzer_state(BuzzerState::On);
  }
  void disabled_buzzer(){
    hal::set_buzzer_state(BuzzerState::Off);
  }

  void toggling_watchdog(){
    hal::toggle_watchdog_state();
  }

  void switch_off_assi_Y(){
    hal::set_assi_Y_state(AssiState::Off);
  }
  void switch_on_assi_Y(){
    hal::set_assi_Y_state(AssiState::On);
  }
  void strobe_assi_Y(){
    hal::set_assi_Y_state(AssiState::Strobe);
  }

  void switch_off_assi_B(){
    hal::set_assi_B_state(AssiState::Off);
  }
  void switch_on_assi_B(){
    hal::set_assi_B_state(AssiState::On);
  }
  void strobe_assi_B(){
    hal::set_assi_B_state(AssiState::Strobe);
  }
}