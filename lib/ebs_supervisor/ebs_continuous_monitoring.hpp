#pragma once
#include <temporal/timer.hpp>
#include <fsm_manager/skeletons.hpp>

using namespace std::chrono_literals;
using namespace as::fsm;


namespace as::ebs_supervisor
{
  class EbsContinousMonitoring
  {
  public:
    static EbsContinousMonitoring& getInstance() {
      static EbsContinousMonitoring instance;
      return instance;
    }

    inline void continuousMonitoring(){
      continousMonitoringAssert([]{return false;}, 50ms, ebsTimer, "PEBS waiting continous monitoring", "PEBS timeout continous monitoring");
      continousMonitoringAssert([]{return false;}, 50ms, sdcTimer, "SDC waiting continous monitoring", "PEBS timeout continous monitoring");
      continousMonitoringAssert([]{return false;}, 50ms, resEmergencyTimer, "Res emergency waiting continous monitoring", "Res emergency timeout continous monitoring");
      continousMonitoringAssert([]{return false;}, 50ms, motorsTimer, "Motors waiting continous monitoring" , "Motora timeout continous monitoring");
    }

  private:
    EbsContinousMonitoring(): ebsTimer(), sdcTimer(), resEmergencyTimer(), motorsTimer() {}
    

    ~EbsContinousMonitoring() = default;
    EbsContinousMonitoring(const EbsContinousMonitoring &) = delete;
    EbsContinousMonitoring &operator=(const EbsContinousMonitoring &) = delete;
    temporal::Timer ebsTimer;
    temporal::Timer sdcTimer;
    temporal::Timer resEmergencyTimer;
    temporal::Timer motorsTimer;
  };
}