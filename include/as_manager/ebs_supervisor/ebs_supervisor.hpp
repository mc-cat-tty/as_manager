#pragma once

#include <as_manager/common/common_types.hpp>
#include <as_manager/fsm_manager/fsm_manager.hpp>

namespace as::ebs_supervisor {
    class EbsSupervisor {
        public:
            static EbsSupervisor& getInstance() {
                static EbsSupervisor instance;
                return instance;
            }

            void run(){
                ebsFsm.run();
            }

        private:
            EbsSupervisor();
            EbsSupervisor(const EbsSupervisor&) = delete;
            EbsSupervisor(EbsSupervisor&&) = delete;
            EbsSupervisor& operator=(const EbsSupervisor&) = delete;
            fsm::Manager<18> ebsFsm;
    };
}