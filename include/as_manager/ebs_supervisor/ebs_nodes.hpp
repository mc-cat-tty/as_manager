#pragma once
#include <as_manager/fsm_manager/nodes_factory.hpp>
#include <as_manager/ebs_supervisor/signal_implementation.hpp>
#include <as_manager/actions/actions.hpp>
#include <as_manager/hal/pin_implementation.hpp>
#include <as_manager/params/parameters.hpp>
#include <as_manager/common/common_types.hpp>
#include <iostream>


namespace as::ebs_supervisor {
    using namespace hal::actions;
    using namespace watchdog;
    using namespace params;

    constexpr auto INIT_PINS_NODE = doActionNode(
      []{
        using namespace hal::pin;
        buzzerPin.setValue(KriaPin::Value::OFF);
        watchdogPin.setValue(KriaPin::Value::OFF);
        ebs1Pin.setValue(KriaPin::Value::ON);
        ebs1Pin.setValue(KriaPin::Value::ON);
        sdcCtrlPin.setValue(KriaPin::Value::OFF);
      },
      "Initialized pins"
    );

    constexpr auto WAIT_ORIN_ON=waitUntilNode(
      []{std::cout<<"VALORE DI ORIN ON: "<<orin_on_signal.get_value()<<std::endl; 
         return orin_on_signal.get_value();}, 
      "ORIN is ON", "Waiting ORIN", [] {}
    );

    constexpr auto WAIT_CANOPEN_ON=waitUntilNode(
      []{ return can_open_on_singal.get_value();}, 
      "CANOPEN is ON", "Waiting CANOPEN", [] {}
    );

    constexpr auto WAIT_ASMS_NODE=waitUntilNode(
      []{return asms_signal.get_value_with_threhold<ValueRespectTreshold::BIGGER>(ASMS_THRESHOLD);}, 
      "ASMS is ON", "Waiting ASMS", [] {}
    );

    constexpr auto START_CANBUS_NODE = doActionNode(
      std::bind(hal::actions::startNode, "canbus_bridge"),
      "Started CAN Bus Bridge"
    );

    constexpr auto START_CANOPEN_NODE = doActionNode(
      std::bind(hal::actions::startNode, "canopen_bridge"),
      "Started CAN Open Bridge"
    );

    constexpr auto START_CONTROL_NODE = doActionNode(
      std::bind(hal::actions::startNode, "control_node"),
      "Started CAN Open Bridge"
    );

    constexpr auto WAIT_MISSION_NODE=waitUntilNode(
      []{return mission_signal.get_value();}, 
      "Mission selected", "Waiting mission", [] {}
    );

    constexpr auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<ebs1_signal.get_value()<<" "<<ebs2_signal.get_value()<<std::endl;
        return ebs1_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold
          and ebs2_signal.get_value() >=  Parameters::getInstance().ebsTankPressureThreshold;
      },
      500ms, "Sufficient EBS tank pressure", "Waiting for sufficient EBS pressure", "EBS pressure timeout"
    );

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureBothActuatorsThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureBothActuatorsThreshold;
      }, 500ms, "Sufficient BRAKE pressure on both actuators", "Waiting sufficient BRAKE pressure on both actuators", "BRAKE pressure on both actuators timeout");

    constexpr auto OPEN_SDC_NODE = doActionNode(open_sdc, "Open SDC");
    constexpr auto CLOSE_SDC_NODE = doActionNode(close_sdc, "Close SDC");

    constexpr auto ASSERT_SDC_OPEN_NODE = assertWithTimeoutNode(
      []{
        return sdc_signal.get_value_with_threhold<ValueRespectTreshold::BIGGER>(SDC_TRESHOLD_OPEN);
      },
       500ms, "SDC open", "Waiting SDC opening", "SDC opening timeout");

    constexpr auto ASSERT_SDC_CLOSE_NODE = assertWithTimeoutNode(
      []{
        return sdc_signal.get_value_with_threhold(SDC_TRESHOLD_CLOSE);
      }, 500ms, "SDC close", "Waiting SDC closing", "SDC closing timeout");
        
    constexpr auto TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().set_toggling();}, "Start toggling watchdog");
    constexpr auto STOP_TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().stop_toggling();}, "Stop toggling watchdog");

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureOneActuatorThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureOneActuatorThreshold;
      }, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient BRAKE pressure", "BRAKE pressure timeout");

    constexpr auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() <=  Parameters::getInstance().unbrakePressureThreshold
          and breake_pressure_front_signal.get_value() <=  Parameters::getInstance().unbrakePressureThreshold;
      }, 500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");

    constexpr auto UNBRAKE_ACT1_NODE = doActionNode(unbrake_act1, "Unbrake Act1");
    constexpr auto BRAKE_ACT1_NODE = doActionNode(brake_act1, "Brake Act1");
    constexpr auto BRAKE_ACT2_NODE = doActionNode(brake_act2, "Brake Act2");
    constexpr auto UNBRAKE_ACT2_NODE = doActionNode(unbrake_act2, "Unbrake Act2");

    constexpr auto WAIT_BRAKE_MOTOR_ENABLED = waitUntilNode(
      []{
        return hal::utils::mask(motors_bit_vector_singal.get_value(), (unsigned)hal::MaxonMotors::CLUTCH);
      }, "Brake motor enabled", "Waiting brake motor enabled", [] {});

    constexpr auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode(brake_with_maxon, "Brake with MAXON motor");

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR = assertWithTimeoutNode(
      []{
        return breake_pressure_rear_signal.get_value() >= Parameters::getInstance().brakePressureMaxonMotorsThreshold
          and breake_pressure_front_signal.get_value() >= Parameters::getInstance().brakePressureMaxonMotorsThreshold;
      }, 500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");

    constexpr auto WAIT_GO_SIGNAL = waitUntilNode<SafetyMonitoringSwitch::DISABLE>(
      []{
        return hal::utils::mask(res_bit_vector_signal.get_value(), (unsigned)hal::Res::GO);
      }, "GO signal received", "Waiting for GO signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });

    constexpr auto WAIT_STOP_SIGNAL =  waitUntilNode<SafetyMonitoringSwitch::DISABLE>(
      []{
        return stop_signal.get_value();
      }, "STOP signal received", "Waiting for STOP signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); }
    );

    constexpr auto WAIT_TS_ACTIVE = waitUntilNode(
      []{
        return rpm_signal.get_value()>2000;
      }, 
      "TS Activated", "Waiting for crank", [] {}
    );

    constexpr auto PULL_CLUTCH_NODE = doActionNode(pullClutch, "Pulling clutch");
    constexpr auto GEAR_FIRST_NODE = doActionNode(setFirstGear, "Setting gear to first");

    constexpr auto ENABLE_MOTORS_NODE = doActionNode(enableMotors, "Enable all motors and for steer do homing before enable");
}