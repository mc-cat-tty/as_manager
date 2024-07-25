#include <fsm_manager/fsm_manager.hpp>
#include <fsm_manager/node_skeletons.hpp>

namespace as::ebs_supervisor {

    using namespace as::fsm;

    EbsSupervisor::EbsSupervisor() : fsm(
        {
            waitUntil([]{ return false; },"Running","Error",) 
        },
        
    ) {

    }
}