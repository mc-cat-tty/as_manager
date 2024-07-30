#pragma once
#include <as_manager/fsm_manager/nodes_factory.hpp>
#include <as_manager/ebs_supervisor/signal_implementation.hpp>
#include <as_manager/actions/actions.hpp>


namespace as::ebs_supervisor {
    using namespace hal::actions;
    using namespace watchdog;
    
    bool waitAsmsAnsMission(){
        return asms_signal.get_value() && mission_signal.get_value();
    }

    constexpr auto WAIT_MISSION_ASMS_NODE=waitUntilNode(
      waitAsmsAnsMission, 
      "Mission selected and ASMS ON", "Waiting for mission and ASMS", [] {}
    );

    constexpr auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<ebs1_signal.get_value()<<" "<<ebs2_signal.get_value()<<std::endl;
        return ebs1_signal.get_value() >= AsManagerNode::getEbsTankPressureThreshold()
          and ebs2_signal.get_value() >=  AsManagerNode::getEbsTankPressureThreshold();
      },
      500ms, "Sufficient EBS tank pressure", "Waiting sufficient PEBS", "PEBS timeout"
    );

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= AsManagerNode::getBrakePressureBothActuatorsThreshold()
          and breake_pressure_front_signal.get_value() >= AsManagerNode::getBrakePressureBothActuatorsThreshold();
      }, 500ms, "Sufficient BRAKE pressure EBS", "Waiting sufficient PBRAKE EBS", "PBRAKE timeout EBS");

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
        return breake_pressure_rear_signal.get_value() >= AsManagerNode::getBrakePressureOneActuatorThreshold()
          and breake_pressure_front_signal.get_value() >= AsManagerNode::getBrakePressureOneActuatorThreshold();
      }, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient PBRAKE", "PBRAKE timeout");

    constexpr auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() <= AsManagerNode::getUnbrakePressureThreshold()
          and breake_pressure_front_signal.get_value() <= AsManagerNode::getUnbrakePressureThreshold();
      }, 500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");

    constexpr auto UNBRAKE_ACT1_NODE = doActionNode(unbrake_act1, "Unbrake Act1");
    constexpr auto BRAKE_ACT1_NODE = doActionNode(brake_act1, "brake Act1");
    constexpr auto BRAKE_ACT2_NODE = doActionNode(brake_act2, "brake Act2");
    constexpr auto UNBRAKE_ACT2_NODE = doActionNode(unbrake_act2, "Unbrake Act2");

    constexpr auto WAIT_BRAKE_MOTOR_ENALBED = waitUntilNode(
      []{
        return hal::utils::motorMask(hal::utils::Motors::Brake, motors_bit_vector_singal.get_value());
      }, "Brake motor enabled", "Waiting brake motor enabled", [] {});

    constexpr auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode(brake_with_maxon, "Brake with Maxon motor");

    constexpr auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR = assertWithTimeoutNode(
      []{
        return breake_pressure_rear_signal.get_value() >= AsManagerNode::getBrakePressureMaxonMotorsThreshold()
          and breake_pressure_front_signal.get_value() >= AsManagerNode::getBrakePressureMaxonMotorsThreshold();
      }, 500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");

    constexpr auto WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return hal::utils::resmask(hal::utils::ResBitVector::Go, res_bit_vector_signal.get_value());
      }, "GO signal received", "Waiting for GO signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });

    constexpr auto WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE =  waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return stop_signal.get_value();
      }, "STOP signal received", "Waiting for STOP signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); }
    );

    constexpr auto WAIT_TS_ACTIVE = waitUntilNode(
      []{
        return rpm_signal.get_value()>2000;
      }, 
      "Mission selected and ASMS ON", "Waiting for mission and ASMS", [] {}
    );

    constexpr auto PULL_CLUTCH_NODE = doActionNode(pullClutch, "Pulling clutch");
    constexpr auto GEAR_FIRST_NODE = doActionNode(setFirstGear, "Setting gear to first");
}