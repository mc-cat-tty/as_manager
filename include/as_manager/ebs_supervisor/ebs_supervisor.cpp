#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/fsm_manager/fsm_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/temporal/common.hpp>
#include <as_manager/ebs_supervisor/ebs_continuous_monitoring.hpp>
#include <as_manager/signals/utils.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_nodes.hpp>
#include <functional>

std::string state;

namespace as::ebs_supervisor {

    EbsSupervisor::EbsSupervisor() :  ebsFsm (
        {
          WAIT_MISSION_ASMS_NODE,
          doActionNode([]{state="CHECKING";}, "Published CHECKING"),
          
          //EBS_CHECK
          ASSERT_EBS_PRESSURE_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE,

          ASSERT_SDC_OPEN_NODE,
          TOGGLING_WATCHDOG_NODE,
          CLOSE_SDC_NODE,
          ASSERT_SDC_CLOSE_NODE,
          STOP_TOGGLING_WATCHDOG_NODE,
          ASSERT_SDC_OPEN_NODE,
          TOGGLING_WATCHDOG_NODE,
          OPEN_SDC_NODE,
          ASSERT_SDC_OPEN_NODE,

          UNBRAKE_ACT1_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          BRAKE_ACT1_NODE,
          UNBRAKE_ACT2_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,

          WAIT_BRAKE_MOTOR_ENALBED,
          UNBRAKE_ACT1_NODE,
          ASSERT_NO_BRAKE_PRESSURE_NODE,
          BRAKE_WITH_MAXON_MOTOR_NODE,
          ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR,

          CLOSE_SDC_NODE,
          WAIT_TS_ACTIVE,

          //READY
          doActionNode([]{state="READY"; assi_manager::AssiManager::getInstance().ready();}, "Published READY"),
          WAIT_GO_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,
          PULL_CLUTCH_NODE,
          GEAR_FIRST_NODE,

          //DRIVING
          doActionNode([]{state="DRIVING"; assi_manager::AssiManager::getInstance().driving();}, "Published DRIVING"),
          WAIT_STOP_SIGNAL_WITH_CONTINUOS_MONITORING_NODE,

          terminalTrapNode(
            []{
              state="FINISHED"; 
              open_sdc(); brake_act1(); brake_act2();
              assi_manager::AssiManager::getInstance().finished();
            }, "FINISHED State"),

        },
        terminalTrapNode(
          []{
            state="Emergency";  
            open_sdc(); brake_act1(); brake_act2();
            assi_manager::AssiManager::getInstance().emergency();
          }, "Emergency State")
    ) {}
}