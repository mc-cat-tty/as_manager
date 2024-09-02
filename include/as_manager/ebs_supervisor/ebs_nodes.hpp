#pragma once
#include <as_manager/fsm_manager/nodes_factory.hpp>
#include <as_manager/ebs_supervisor/signal_implementation.hpp>
#include <as_manager/ebs_supervisor/ebs_nodes_helpers.hpp>
#include <as_manager/actions/actions.hpp>
#include <as_manager/hal/pin_implementation.hpp>
#include <as_manager/params/parameters.hpp>
#include <as_manager/common/common_types.hpp>
#include <as_manager/timing/timer.hpp>
#include <iostream>


namespace as::ebs_supervisor {
    using namespace hal::actions;
    using namespace watchdog;
    using namespace params;
    using namespace std::chrono_literals;

    constexpr auto WAIT_500_MS_NODE = sleepNode(500ms);
    constexpr auto WAIT_1000_MS_NODE = sleepNode(1000ms);
    constexpr auto WAIT_1500_MS_NODE = sleepNode(1500ms);
    constexpr auto WAIT_2000_MS_NODE = sleepNode(2000ms);
    constexpr auto WAIT_5_S_NODE = sleepNode<SafetyMonitoringSwitch::ENABLE>(5000ms);

    constexpr auto INIT_PINS_NODE = doActionNode(
      []{
        using namespace hal;
        set_buzzer_state(KriaPin::Value::OFF);
        toggle_sdc_state(SdcState::CLOSE);
        Watchdog::getInstance().set_toggling();

        if (Parameters::getInstance().safetyFeatures) {
          toggle_actuator1_state(ActuatorState::BRAKING);
          toggle_actuator2_state(ActuatorState::BRAKING);
        }
        else {
          toggle_actuator1_state(ActuatorState::UNBRAKING);
          toggle_actuator2_state(ActuatorState::UNBRAKING);
        }
      },
      "Initialized pins"
    );

    constexpr auto WAIT_ORIN_ON_NODE = waitUntilNode(
      hal::read_orin_on,
      "ORIN is ON", "Waiting ORIN", [] {}
    );

    constexpr auto WAIT_CANOPEN_ON_NODE = waitUntilNode(
      hal::read_can_open_on, 
      "CANOPEN is ON", "Waiting CANOPEN", [] {}
    );

    constexpr auto WAIT_ASMS_NODE=waitUntilNode(
      hal::read_asms_status, 
      "ASMS is ON", "Waiting ASMS ON", [] {}
    );

    constexpr auto START_CANBUS_NODE = doActionNode(
      std::bind(hal::actions::startNode, "canbus_bridge", ""),
      "Started CAN Bus Bridge"
    );

    constexpr auto START_CANOPEN_NODE = doActionNode(
      std::bind(hal::actions::startNode, "canopen_bridge", ""),
      "Started CAN Open Bridge"
    );

    auto START_CONTROL_NODE = doActionNode(
      [] {
        hal::actions::startNode(
          "control_node",
          static_cast<std::string>(COCKPIT::CockpitMissionLookup.at(static_cast<COCKPIT::MMR_MISSION_VALUE>(AsManagerNode::getMission())))
        );
      },
      "Started CAN Open Bridge"
    );

    constexpr auto WAIT_MISSION_NODE=waitUntilNode(
      hal::is_autonomous_mission, 
      "Mission selected", "Waiting mission", [] {}
    );

    constexpr auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<ebs1_signal.get_value()<<" "<<ebs2_signal.get_value()<<std::endl;
        return ebs1_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold
          and ebs2_signal.get_value() >=  Parameters::getInstance().ebsTankPressureThreshold;
      },
      200ms, "Sufficient EBS tank pressure", "Waiting for sufficient EBS pressure", "EBS pressure timeout"
    );

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureBothActuatorsThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureBothActuatorsThreshold;
      },
      200ms, "Sufficient BRAKE pressure on both actuators", "Waiting sufficient BRAKE pressure on both actuators", "BRAKE pressure on both actuators timeout");

    constexpr auto OPEN_SDC_NODE = doActionNode(open_sdc, "Open SDC");
    constexpr auto CLOSE_SDC_NODE = doActionNode(close_sdc, "Close SDC");

    constexpr auto ASSERT_SDC_OPEN_NODE = assertWithTimeoutNode(
      []{
        return hal::read_sdc() == hal::SdcState::OPEN;
      },
      100ms, "SDC open", "Waiting SDC opening", "SDC opening timeout");

    constexpr auto ASSERT_SDC_CLOSE_NODE = assertWithTimeoutNode(
      []{
        return hal::read_sdc() == hal::SdcState::CLOSE;
      },
      100ms, "SDC close", "Waiting SDC closing", "SDC closing timeout");
        
    constexpr auto TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().set_toggling();}, "Start toggling watchdog");
    constexpr auto STOP_TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().stop_toggling();}, "Stop toggling watchdog");

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureOneActuatorThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureOneActuatorThreshold;
      },
      200ms, "Sufficient BRAKE pressure", "Waiting sufficient BRAKE pressure", "BRAKE pressure timeout");

    constexpr auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() <=  Parameters::getInstance().unbrakePressureThreshold
          and breake_pressure_front_signal.get_value() <=  Parameters::getInstance().unbrakePressureThreshold;
      },
      500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");

    constexpr auto UNBRAKE_ACT1_NODE = doActionNode(unbrake_act1, "Unbrake Act1");
    constexpr auto BRAKE_ACT1_NODE = doActionNode(brake_act1, "Brake Act1");
    constexpr auto BRAKE_ACT2_NODE = doActionNode(brake_act2, "Brake Act2");
    constexpr auto UNBRAKE_ACT2_NODE = doActionNode(unbrake_act2, "Unbrake Act2");

    constexpr auto WAIT_BRAKE_AND_CLUCTH_MOTORS_ENABLED_NODE = waitUntilNode(
      []{
        return hal::utils::mask(hal::read_motors_bit_vector(), (unsigned)hal::MaxonMotors::CLUTCH | (unsigned)hal::MaxonMotors::BRAKE);
      }, "Brake and clutch motors enabled", "Waiting brake and clutch motor enabled", [] {});

    constexpr auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode(brake_with_maxon, "Brake with MAXON motor");

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR = assertWithTimeoutNode(
      []{
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureMaxonMotorsThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureMaxonMotorsThreshold;
      },
      500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");

    constexpr auto WAIT_GO_SIGNAL_OFF_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return !hal::utils::mask(hal::read_res_bit_vector(), (unsigned) hal::Res::GO);
      }, "GO signal OFF received", "Waiting for GO signal OFF", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });

    constexpr auto WAIT_GO_SIGNAL_ON_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return hal::utils::mask(hal::read_res_bit_vector(), (unsigned) hal::Res::GO);
      }, "GO signal ON received", "Waiting for GO signal ON", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });

    constexpr auto WAIT_STOP_SIGNAL =  waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      hal::read_stop_message, "STOP signal received", "Waiting for STOP signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); }
    );

    constexpr auto WAIT_TS_ACTIVE_NODE = waitUntilNode(
      []{
        return rpm_signal.get_value() > 3000;
      }, 
      "TS Activated", "Waiting for crank", [] {}
    );

    constexpr auto PULL_CLUTCH_NODE = doActionNode(pullClutch, "Pulling clutch");
    constexpr auto GEAR_FIRST_NODE = doActionNode(setFirstGear, "Setting gear to first");

    constexpr auto ENABLE_MOTORS_NODE = doActionNode(enableMotors, "Enable all motors and for steer do homing before enable");
}