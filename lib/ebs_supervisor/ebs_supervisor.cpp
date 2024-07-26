#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/nodes.hpp>
#include <ebs_supervisor/ebs_supervisor.hpp>
#include <temporal/common.hpp>
#include <ebs_supervisor/signal_implementation.hpp>
#include <ebs_supervisor/ebs_continuous_monitoring.hpp>
#include <functional>

std::string state;

namespace as::ebs_supervisor {
    using namespace fsm;
    using namespace std::chrono_literals;

    //pf = thresholdDecorate(std::bind(Signal::get_val, obj), 90)
    // inline auto ebs_cm= EbsContinousMonitoring::getInstance();

    constexpr inline const auto WAIT_MISSION_ASMS_NODE=waitUntilNode([]{return false;}, "Mission selected and ASMS ON", "Waiting for mission and ASMS", [] {});
    constexpr inline const auto ASSERT_EBS_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient EBS tank pressure", "Waiting sufficient PEBS", "PEBS timeout");
    constexpr inline const auto OPEN_SDC_NODE = doActionNode([]{}, "Open SDC");
    constexpr inline const auto CLOSE_SDC_NODE = doActionNode([]{}, "Close SDC");
    constexpr inline const auto ASSERT_SDC_OPEN_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "SDC open", "Waiting SDC opening", "SDC opening timeout");
    constexpr inline const auto ASSERT_SDC_CLOSE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "SDC close", "Waiting SDC closing", "SDC closing timeout");
    constexpr inline const auto TOGGLING_WATCHDOG_NODE = doActionNode([]{}, "Start toggling watchdog");
    constexpr inline const auto STOP_TOGGLING_WATCHDOG_NODE = doActionNode([]{}, "Stop toggling watchdog");
    constexpr inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient PBRAKE", "PBRAKE timeout");
    constexpr inline const auto ASSERT_NO_BRAKE_PRESSURE_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "No brake pressure", "Waiting no brake pressure", "Brake pressure timeout");
    constexpr inline const auto UNBRAKE_ACT1_NODE = doActionNode([]{}, "Unbrake Act1");
    constexpr inline const auto BRAKE_ACT1_NODE = doActionNode([]{}, "brake Act1");
    constexpr inline const auto BRAKE_ACT2_NODE = doActionNode([]{}, "brake Act2");
    constexpr inline const auto UNBRAKE_ACT2_NODE = doActionNode([]{}, "Unbrake Act2");
    constexpr inline const auto WAIT_BRAKE_MOTOR_ENALBED = waitUntilNode([]{return false;}, "Brake motor enabled", "Waiting for brake motor enable", [] {});
    constexpr inline const auto BRAKE_WITH_MAXON_MOTOR_NODE = doActionNode([]{}, "Brake with Maxon motor");   
    constexpr inline const auto ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR_NODE = assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient BRAKE pressure with MAXON", "Waiting sufficient brake pressure with MAXON", "Brake pressure MAXON timeout");
    constexpr inline const auto WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE = waitUntilNode<SafetyMonitoringSwitch::ENABLE>([]{return false;}, "GO signal received", "Waiting for GO signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });
    constexpr inline const auto WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE =  waitUntilNode<SafetyMonitoringSwitch::ENABLE>([]{return false;}, "STOP signal received", "Waiting for STOP signal", [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); });
    constexpr inline const auto FINISH_NODE = terminalTrapNode([]{state="FINISHED";}, "FINISHED State");

    EbsSupervisor::EbsSupervisor() :  ebsFsm (
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