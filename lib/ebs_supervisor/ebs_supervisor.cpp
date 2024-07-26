#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/node_skeletons.hpp>
#include <ebs_supervisor/ebs_supervisor.hpp>
#include <functional>

namespace as::ebs_supervisor {
    using namespace fsm;

    EbsSupervisor::EbsSupervisor() : ebsFsm (
        {
          []() -> NodeFlowCtrl {},
          []() -> NodeFlowCtrl {}
        },
        []() -> NodeFlowCtrl {}
    ) {}
}