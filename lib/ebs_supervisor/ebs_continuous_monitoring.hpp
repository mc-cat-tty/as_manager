#pragma once
#include <temporal/timer.hpp>

namespace as::ebs_supervisor{

    class EbsContinousMonitoring {
        public:
            EbsContinousMonitoring();
            void continousMonitoring();
        private:
            temporal::Timer ebsTimer;
            temporal::Timer sdcTimer;
            temporal::Timer resEmergencyTimer;
            temporal::Timer motorsTimer;

    };
}