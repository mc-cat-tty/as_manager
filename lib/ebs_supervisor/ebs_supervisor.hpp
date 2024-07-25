#pragma once

#include <common/common_type.hpp>

namespace as::ebs_supervisor {

    class EbsSupervisor {

        public:

            EbsSupervisor();

            void  run(){
                fsm.run();
            }

        private:
            as::fsm::Manager<28> fsm;

    };
    
}