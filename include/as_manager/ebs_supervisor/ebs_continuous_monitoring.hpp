#pragma once
#include <as_manager/timing/timer.hpp>
#include <as_manager/fsm_manager/skeletons.hpp>
#include <as_manager/ebs_supervisor/signal_implementation.hpp>
#include <as_manager/common/common_types.hpp>
#include <as_manager/hal/utils.hpp>
#include <as_manager/params/parameters.hpp>
#include <as_manager/as_manager.hpp>


namespace as::ebs_supervisor {
  using namespace std::chrono_literals;
  using namespace as::fsm;
  using namespace params;

  class EbsContinousMonitoring {
  public:
    static EbsContinousMonitoring& getInstance() {
      static EbsContinousMonitoring instance;
      return instance;
    }

    inline void continuousMonitoring(){
      continousMonitoringAssert(
        []{
          return ebs1_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold and
          ebs2_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold;
        }, 
        50ms, ebsTimer,
        "PEBS waiting continous monitoring", "PEBS timeout continous monitoring"
      );
      
      continousMonitoringAssert(
        []{
          return sdc_signal.get_value_with_threhold(SDC_TRESHOLD_CLOSE);
        },
        50ms, sdcTimer,
        "SDC waiting continous monitoring", "SDC timeout continous monitoring"
      );
      
      continousMonitoringAssert(
        []{
          return static_cast<hal::ResState>(res_emergency_signal.get_value()) == hal::ResState::OPERATIONAL;
        },
         50ms, resEmergencyTimer,
         "Res emergency waiting continous monitoring", "Res emergency timeout continous monitoring"
      );
    }

  private:
    EbsContinousMonitoring(): ebsTimer(), sdcTimer(), resEmergencyTimer(), motorsTimer() {}

    ~EbsContinousMonitoring() = default;
    EbsContinousMonitoring(const EbsContinousMonitoring &) = delete;
    EbsContinousMonitoring &operator=(const EbsContinousMonitoring &) = delete;
    timing::TimerAsync ebsTimer, sdcTimer, resEmergencyTimer, motorsTimer;
  };
}