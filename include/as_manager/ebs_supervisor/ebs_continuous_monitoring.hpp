#pragma once
#include <as_manager/timing/timer.hpp>
#include <as_manager/fsm_manager/skeletons.hpp>
#include <as_manager/ebs_supervisor/signal_implementation.hpp>
#include <as_manager/hal/utils.hpp>

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
      continousMonitoringAssert(
        []{
            return ebs1_signal.get_value() >= EXPECTED_PRESSURE_EBS_TANK and
            ebs2_signal.get_value() >= EXPECTED_PRESSURE_EBS_TANK;
        }, 
        50ms, ebsTimer, "PEBS waiting continous monitoring", "PEBS timeout continous monitoring");
      continousMonitoringAssert(
        []{
          return sdc_signal.get_value_with_threhold(SDC_TRESHOLD_CLOSE);
        },
        50ms, sdcTimer, "SDC waiting continous monitoring", "SDC timeout continous monitoring");
      continousMonitoringAssert(
        []{
          return res_emergency_signal.get_value();
        },
         50ms, resEmergencyTimer, "Res emergency waiting continous monitoring", "Res emergency timeout continous monitoring");
      continousMonitoringAssert(
        []{
          return hal::utils::motorMask(hal::utils::Motors::All, motors_bit_vector_singal.get_value());
        }, 
        50ms, motorsTimer, "Motors waiting continous monitoring" , "Motors timeout continous monitoring");
    }

  private:
    EbsContinousMonitoring(): ebsTimer(), sdcTimer(), resEmergencyTimer(), motorsTimer() {}
    

    ~EbsContinousMonitoring() = default;
    EbsContinousMonitoring(const EbsContinousMonitoring &) = delete;
    EbsContinousMonitoring &operator=(const EbsContinousMonitoring &) = delete;
    timing::TimerAsync ebsTimer;
    timing::TimerAsync sdcTimer;
    timing::TimerAsync resEmergencyTimer;
    timing::TimerAsync motorsTimer;
  };
}