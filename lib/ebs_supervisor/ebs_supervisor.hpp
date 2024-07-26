#pragma once

#include <common/common_type.hpp>

namespace as::ebs_supervisor {
    class EbsSupervisor {
        public:
            EbsSupervisor();

            void run(){
                ebsFsm.run();
            }

        private:
            fsm::Manager<29> ebsFsm;
    };
}