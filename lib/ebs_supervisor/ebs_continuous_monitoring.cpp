#include <ebs_supervisor/ebs_continuous_monitoring.hpp>
#include <fsm_manager/skeletons.hpp>

namespace as::ebs_supervisor{

    using namespace std::chrono_literals;
    using namespace fsm;

    EbsContinousMonitoring::EbsContinousMonitoring(): ebsTimer(), sdcTimer(), resEmergencyTimer(), motorsTimer() {}

    void EbsContinousMonitoring::continousMonitoring(){
        continousMonitoringAssert([]{return false;}, 50ms, ebsTimer, "PEBS waiting continous monitoring", "PEBS timeout continous monitoring");
        continousMonitoringAssert([]{return false;}, 50ms, sdcTimer, "SDC waiting continous monitoring", "PEBS timeout continous monitoring");
        continousMonitoringAssert([]{return false;}, 50ms, resEmergencyTimer, "Res emergency waiting continous monitoring", "Res emergency timeout continous monitoring");
        continousMonitoringAssert([]{return false;}, 50ms, motorsTimer, "Motors waiting continous monitoring" , "Motora timeout continous monitoring");
    }
}