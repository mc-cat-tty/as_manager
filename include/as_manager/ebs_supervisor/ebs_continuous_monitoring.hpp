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

    inline void continuousMonitoring() {
      continousMonitoringAssert(
        []{
          return hal::read_sdc() == hal::SdcState::CLOSE;
        },
        50ms, sdcTimer,
        "Waiting SDC for continous monitoring", "SDC timeout continous monitoring"
      );

      if (not Parameters::getInstance().safetyFeatures) return;
      
      continousMonitoringAssert(
        []{
          return ebs1_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold and
          ebs2_signal.get_value() >= Parameters::getInstance().ebsTankPressureThreshold;
        }, 
        50ms, ebsTimer,
        "Waiting PEBS for continous monitoring", "PEBS timeout continous monitoring"
      );
      
      continousMonitoringAssert(
        []{
          return hal::read_res_state() == hal::ResState::OPERATIONAL;
        },
         50ms, resEmergencyTimer,
         "Waiting RES emergency for continous monitoring", "Res emergency timeout continous monitoring"
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