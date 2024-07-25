#include <actions/actions.hpp>

namespace hal::action {
  void Actions::open_sdc(){
    hal.toggle_sdc_state(SdcState::Open);
  }
  void Actions::close_sdc(){
    hal.toggle_sdc_state(SdcState::Closed);
  }

  void Actions::brake_act1(){
    hal.toggle_actuator1_state(ActuatorState::Braking);
  }

  void Actions::brake_act2(){
    hal.toggle_actuator2_state(ActuatorState::Braking);
  }

  void Actions::brake_all_act(){
    hal.toggle_actuator1_state(ActuatorState::Braking);
    hal.toggle_actuator2_state(ActuatorState::Braking);
  }

  void Actions::unbrake_act1(){
    hal.toggle_actuator1_state(ActuatorState::Unbrake);
  }
  void Actions::unbrake_act2(){
    hal.toggle_actuator2_state(ActuatorState::Unbrake);
  }
  void Actions::unbrake_all_act(){
    hal.toggle_actuator1_state(ActuatorState::Unbrake);
    hal.toggle_actuator2_state(ActuatorState::Unbrake);
  }

  void Actions::active_buzzer(){
    hal.set_buzzer_state(BuzzerState::On);
  }
  void Actions::disabled_buzzer(){
    hal.set_buzzer_state(BuzzerState::Off);
  }

  void Actions::toggling_watchdog(){
    hal.toggle_watchdog_state();
  }

  void Actions::switch_off_assi_Y(){
    hal.set_assi_Y_state(AssiState::Off);
  }
  void Actions::switch_on_assi_Y(){
    hal.set_assi_Y_state(AssiState::On);
  }
  void Actions::strobe_assi_Y(){
    hal.set_assi_Y_state(AssiState::Strobe);
  }

  void Actions::switch_off_assi_B(){
    hal.set_assi_B_state(AssiState::Off);
  }
  void Actions::switch_on_assi_B(){
    hal.set_assi_B_state(AssiState::On);
  }
  void Actions::strobe_assi_B(){
    hal.set_assi_B_state(AssiState::Strobe);
  }
}