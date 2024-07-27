#include <watchdog/watchdog.hpp>
#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/nodes.hpp>
#include <ebs_supervisor/ebs_supervisor.hpp>
#include <temporal/common.hpp>
#include <ebs_supervisor/ebs_continuous_monitoring.hpp>
#include <ebs_supervisor/signal_implementation.hpp>
#include <actions/actions.hpp>
#include <signals/utils.hpp>
#include <hal/utils.hpp>
#include <assi_manager/assi_manager.hpp>
#include <functional>

std::string state;

namespace as::ebs_supervisor {
    using namespace fsm;
    using namespace std::chrono_literals;
    using namespace signals::utils;
    using namespace hal::actions;
    using namespace hal;
    using namespace watchdog;

    bool waitAsmsAnsMission(){
        return asms_signal.get_value() && mission_signal.get_value();
    }

    constexpr inline const auto WAIT_MISSION_ASMS_NODE=waitUntilNode(
      waitAsmsAnsMission, 
      "Mission selected and ASMS ON", "Waiting for mission and ASMS", [] {}
    );

    constexpr inline const auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<ebs1_signal.get_value()<<" "<<ebs2_signal.get_value()<<std::endl;
        return ebs1_signal.get_value() >= EXPECTED_PRESSURE_EBS_TANK
          and ebs2_signal.get_value() >= EXPECTED_PRESSURE_EBS_TANK;
      },
      500ms, "Sufficient EBS tank pressure", "Waiting sufficient PEBS", "PEBS timeout"
    );

    constexpr inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_BOTH_ACTUATORS
          and breake_pressure_front_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_BOTH_ACTUATORS;
      }, 500ms, "Sufficient BRAKE pressure EBS", "Waiting sufficient PBRAKE EBS", "PBRAKE timeout EBS");

    constexpr inline const auto OPEN_SDC_NODE = doActionNode(open_sdc, "Open SDC");
    constexpr inline const auto CLOSE_SDC_NODE = doActionNode(close_sdc, "Close SDC");

    constexpr inline const auto ASSERT_SDC_OPEN_NODE = assertWithTimeoutNode(
      []{
        return sdc_signal.get_value_with_threhold<ValueRespectTreshold::BIGGER>(SDC_TRESHOLD_OPEN);
      },
       500ms, "SDC open", "Waiting SDC opening", "SDC opening timeout");

    constexpr inline const auto ASSERT_SDC_CLOSE_NODE = assertWithTimeoutNode(
      []{
        return sdc_signal.get_value_with_threhold(SDC_TRESHOLD_CLOSE);
      }, 500ms, "SDC close", "Waiting SDC closing", "SDC closing timeout");
        
    constexpr inline const auto TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().set_toggling();}, "Start toggling watchdog");
    constexpr inline const auto STOP_TOGGLING_WATCHDOG_NODE = doActionNode([]{Watchdog::getInstance().stop_toggling();}, "Stop toggling watchdog");

    constexpr inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_ONE_ACTUATOR
          and breake_pressure_front_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_ONE_ACTUATOR;
      }, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient PBRAKE", "PBRAKE timeout");

    constexpr inline const auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode(
      []{
        std::cout<<"PBRAKE: "<<breake_pressure_rear_signal.get_value()<<" "<<breake_pressure_front_signal.get_value()<<std::endl;
        return breake_pressure_rear_signal.get_value() <= EXPECTED_UNBRAKE_PRESSURE
          and breake_pressure_front_signal.get_value() <= EXPECTED_UNBRAKE_PRESSURE;
      }, 500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");

    constexpr inline const auto UNBRAKE_ACT1_NODE = doActionNode(unbrake_act1, "Unbrake Act1");
    constexpr inline const auto BRAKE_ACT1_NODE = doActionNode(brake_act1, "brake Act1");
    constexpr inline const auto BRAKE_ACT2_NODE = doActionNode(brake_act2, "brake Act2");
    constexpr inline const auto UNBRAKE_ACT2_NODE = doActionNode(unbrake_act2, "Unbrake Act2");

    constexpr inline const auto WAIT_BRAKE_MOTOR_ENALBED = waitUntilNode(
      []{
        return hal::utils::motorMask(hal::utils::Motors::Brake, motors_bit_vector_singal.get_value());
      }, "Brake motor enabled", "Waiting brake motor enabled", [] {});

    constexpr inline const auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode(brake_with_maxon, "Brake with Maxon motor");

    constexpr inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR = assertWithTimeoutNode(
      []{
        return breake_pressure_rear_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_MAXON_MOTOR
          and breake_pressure_front_signal.get_value() >= EXPECTED_BRAKE_PRESSURE_MAXON_MOTOR;
      }, 500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");

    constexpr inline const auto WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return hal::utils::resmask(hal::utils::ResBitVector::Go, res_bit_vector_signal.get_value());
      }, "GO signal received", "Waiting for GO signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });

    constexpr inline const auto WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE =  waitUntilNode<SafetyMonitoringSwitch::ENABLE>(
      []{
        return stop_signal.get_value();
      }, "STOP signal received", "Waiting for STOP signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); }
    );

    constexpr inline const auto WAIT_TS_ACTIVE = waitUntilNode(
      []{
        return rpm_signal.get_value()>2000;
      }, 
      "Mission selected and ASMS ON", "Waiting for mission and ASMS", [] {}
    );

    EbsSupervisor::EbsSupervisor() :  ebsFsm (
        {
          WAIT_MISSION_ASMS_NODE,
          doActionNode([]{state="CHECKING";}, "Published CHECKING"),
          //EBS_CHECK
          ASSERT_EBS_PRESSURE_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE,

          ASSERT_SDC_OPEN_NODE,  // O O
          TOGGLING_WATCHDOG_NODE,
          CLOSE_SDC_NODE,
          ASSERT_SDC_CLOSE_NODE,  // C C
          STOP_TOGGLING_WATCHDOG_NODE,
          ASSERT_SDC_OPEN_NODE,  // O C
          TOGGLING_WATCHDOG_NODE,
          OPEN_SDC_NODE,
          ASSERT_SDC_OPEN_NODE,  // C O

          UNBRAKE_ACT1_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          BRAKE_ACT1_NODE,
          UNBRAKE_ACT2_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,

          WAIT_BRAKE_MOTOR_ENALBED,
          UNBRAKE_ACT1_NODE,
          ASSERT_NO_BRAKE_PRESSURE_NODE,
          BRAKE_WITH_MAXON_MOTOR_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR,

          CLOSE_SDC_NODE,
          WAIT_TS_ACTIVE,
          //READY
          doActionNode([]{state="READY";assi_manager::AssiManager::getInstance().enableAssiY();}, "Published READY"),
          WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,
          //DRIVING
          doActionNode([]{state="DRIVING"; assi_manager::AssiManager::getInstance().enableStrobeAssiY();}, "Published DRIVING"),
          WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,
          terminalTrapNode(
            []{
              state="FINISHED"; 
              open_sdc(); brake_act1(); brake_act2();
              assi_manager::AssiManager::getInstance().enableAssiB(); 
            }, "FINISHED State"),

        },
        terminalTrapNode(
          []{
            state="Emergency";  
            open_sdc(); brake_act1(); brake_act2(); 
            assi_manager::AssiManager::getInstance().enableStrobeAssiB(); assi_manager::AssiManager::getInstance().enableBuzzer();
          }, "Emergency State")
    ) {}
}