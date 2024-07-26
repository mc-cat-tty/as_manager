#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/nodes.hpp>
#include <ebs_supervisor/ebs_supervisor.hpp>
#include <temporal/common.hpp>
#include <functional>

std::string state;

namespace as::ebs_supervisor {
    using namespace fsm;
    using namespace std::chrono_literals;

    EbsSupervisor::EbsSupervisor() : ebsFsm (
        {
          waitUntilNode([]{return false;}, "Mission selected and ASMS ON", "Waiting for mission and ASMS"),
          doActionNode([]{state="CHECKING";}, "Published CHECKING"),
          assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient EBS tank pressure", "Waiting sufficient PEBS", "PEBS timeout"),
          assertWithTimeoutNode([]{return false;}, 500ms, "Sufficient BRAKE pressure", "Waiting sufficient PBRAKE", "PBRAKE timeout"),
          assertWithTimeoutNode([]{return false;}, 500ms, "SDC open pressure", "Waiting SDC opening", "SDC opening timeout"),
          doActionNode([]{}, "Start toggling watchdog"),
          doActionNode([]{}, "Close SDC"),
          assertWithTimeoutNode([]{return false;}, 500ms, "SDC is close", "Waiting SDC closing", "SDC closing timeout"),
          doActionNode([]{}, "Stop toggling watchdog"),
          assertWithTimeoutNode([]{return false;}, 500ms, "SDC is open", "Waiting SDC opening", "SDC opening timeout")
        },
        terminalTrapNode([]{state="Emergency";}, "Emergency State")
    ) {}
}