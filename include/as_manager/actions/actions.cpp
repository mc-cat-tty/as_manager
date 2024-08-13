#include <as_manager/actions/actions.hpp>
#include <iostream>

namespace hal::actions {
  void open_sdc(){
    hal::toggle_sdc_state(SdcState::OPEN);
  }
  void close_sdc(){
    hal::toggle_sdc_state(SdcState::CLOSE);
  }

  void brake_act1(){
    hal::toggle_actuator1_state(ActuatorState::BRAKING);
  }

  void brake_act2(){
    hal::toggle_actuator2_state(ActuatorState::BRAKING);
  }

  void brake_all_act(){
    hal::toggle_actuator1_state(ActuatorState::BRAKING);
    hal::toggle_actuator2_state(ActuatorState::BRAKING);
  }

  void unbrake_act1(){
    hal::toggle_actuator1_state(ActuatorState::UNBRAKING);
  }
  
  void unbrake_act2(){
    hal::toggle_actuator2_state(ActuatorState::UNBRAKING);
  }

  void unbrake_all_act(){
    hal::toggle_actuator1_state(ActuatorState::UNBRAKING);
    hal::toggle_actuator2_state(ActuatorState::UNBRAKING);
  }

  void active_buzzer(){
    hal::set_buzzer_state(BuzzerState::ON);
  }

  void disabled_buzzer(){
    hal::set_buzzer_state(BuzzerState::OFF);
  }

  void switch_off_assi_Y(){
    hal::set_assi_Y_state(AssiState::OFF);
  }
  void switch_on_assi_Y(){
    hal::set_assi_Y_state(AssiState::ON);
  }
  void strobe_assi_Y(){
    hal::set_assi_Y_state(AssiState::STROBE);
  }

  void brake_with_maxon(){
    hal::send_brake_pressure_percentage(-0.008f);
  }

  void switch_off_assi_B(){
    hal::set_assi_B_state(AssiState::OFF);
  }

  void switch_on_assi_B(){
    hal::set_assi_B_state(AssiState::ON);
  }
  
  void strobe_assi_B(){
    hal::set_assi_B_state(AssiState::STROBE);
  }

  void pullClutch() {
    hal::pull_clutch();
  }

  void setFirstGear() {
    hal::set_gear(1);
  }

  void enableMotors() {
    hal::enable_motors();
  }

  void startNode(std::string nodeName) {
    pid_t newNodePid = fork();

    if (newNodePid < 0) {
      std::cerr << "Fork failed with error number: " << newNodePid << std::endl;
      exit(255);
    }
    if (!newNodePid) {
      auto launchFile = nodeName + "_launch.py";
      int ret = execlp("ros2", "ros2", "launch", nodeName.c_str(), launchFile.c_str() , (char*)NULL);
      if (ret == -1) {
        std::cerr << "Exec failed with error number: " << ret << std::endl;
        exit(4);
      }
    }
    else return;
  }
}