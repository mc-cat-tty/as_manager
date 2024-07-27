#pragma once

#include <common/common_type.hpp>
#include <fsm_manager/fsm_manager.hpp>

namespace as::ebs_supervisor {
    class EbsSupervisor {
        public:
            EbsSupervisor();

            void run(){
                ebsFsm.run();
            }

        private:
            fsm::Manager<31> ebsFsm;
    };
}