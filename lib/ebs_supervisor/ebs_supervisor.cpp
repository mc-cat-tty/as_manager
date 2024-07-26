#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/nodes.hpp>
#include <ebs_supervisor/ebs_supervisor.hpp>
#include <temporal/common.hpp>

#include <functional>

std::string state;

namespace as::ebs_supervisor {
    using namespace fsm;
    using namespace std::chrono_literals;

    inline const auto WAIT_MISSION_ASMS_NODE=waitUntilNode([]{return false;}, "Mission selected and ASMS ON", "Waiting for mission and ASMS");
    inline const auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient EBS tank pressure", "Waiting sufficient PEBS", "PEBS timeout");
    inline const auto OPEN_SDC_NODE = doActionNode([]{}, "Open SDC");
    inline const auto CLOSE_SDC_NODE = doActionNode([]{}, "Close SDC");
    inline const auto ASSERT_SDC_OPEN_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "SDC open", "Waiting SDC opening", "SDC opening timeout");
    inline const auto ASSERT_SDC_CLOSE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "SDC close", "Waiting SDC closing", "SDC closing timeout");
    inline const auto TOGGLING_WATCHDOG_NODE = doActionNode([]{}, "Start toggling watchdog");
    inline const auto STOP_TOGGLING_WATCHDOG_NODE = doActionNode([]{}, "Stop toggling watchdog");
    inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient PBRAKE", "PBRAKE timeout");
    inline const auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");
    inline const auto UNBRAKE_ACT1_NODE = doActionNode([]{}, "Unbrake Act1");
    inline const auto BRAKE_ACT1_NODE = doActionNode([]{}, "brake Act1");
    inline const auto BRAKE_ACT2_NODE = doActionNode([]{}, "brake Act2");
    inline const auto UNBRAKE_ACT2_NODE = doActionNode([]{}, "Unbrake Act2");
    inline const auto WAIT_BRAKE_MOTOR_ENALBED = waitUntilNode([]{return false;}, "Brake motor enabled", "Waiting for brake motor enable");
    inline const auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode([]{}, "Brake with Maxon motor");   
    inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");
    inline const auto WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>([]{return false;}, "GO signal received", "Waiting for GO signal", []{});
    inline const auto WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE =  waitUntilNode<SafetyMonitoringSwitch::ENABLE>([]{return false;}, "STOP signal received", "Waiting for STOP signal", []{});
    inline const auto FINISH_NODE = terminalTrapNode([]{state="FINISHED";}, "FINISHED State");

    EbsSupervisor::EbsSupervisor() : ebsFsm (
        {
          WAIT_MISSION_ASMS_NODE,
          doActionNode([]{state="CHECKING";}, "Published CHECKING"),
          ASSERT_EBS_PRESSURE_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          ASSERT_SDC_OPEN_NODE,
          TOGGLING_WATCHDOG_NODE,
          CLOSE_SDC_NODE,
          ASSERT_SDC_CLOSE_NODE,
          STOP_TOGGLING_WATCHDOG_NODE,
          ASSERT_SDC_OPEN_NODE,
          TOGGLING_WATCHDOG_NODE,
          ASSERT_SDC_CLOSE_NODE,
          OPEN_SDC_NODE,
          ASSERT_SDC_OPEN_NODE,
          UNBRAKE_ACT1_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          BRAKE_ACT1_NODE,
          UNBRAKE_ACT2_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          WAIT_BRAKE_MOTOR_ENALBED,
          UNBRAKE_ACT1_NODE,
          ASSERT_NO_BRAKE_PRESSURE_NODE,
          BRAKE_WITH_MAXON_MOTOR_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR_NODE,
          doActionNode([]{state="READY";}, "Published READY"),
          WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,
          doActionNode([]{state="DRIVING";}, "Published DRIVING"),
          WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,
          FINISH_NODE,

        },
        {terminalTrapNode([]{state="Emergency";}, "Emergency State")}
    ) {}
}