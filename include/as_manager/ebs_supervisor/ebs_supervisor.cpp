#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/fsm_manager/fsm_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/ebs_supervisor/ebs_continuous_monitoring.hpp>
#include <as_manager/signals/utils.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_nodes.hpp>
#include <as_manager/as_manager.hpp>
#include <functional>

std::string state;

namespace as::ebs_supervisor {

    EbsSupervisor::EbsSupervisor() :  ebsFsm (
        {
          // CANBUS BRIDGE as the first thing
          START_CANBUS_NODE,

          doActionNode(std::bind(hal::send_current_state, AsState::OFF), "Published OFF"),
          
          // OFF
          INIT_PINS_NODE,
          // WAIT_ASMS_NODE,
          // START_CANOPEN_NODE,
          // WAIT_MISSION_NODE,
          doActionNode(std::bind(hal::send_current_state, AsState::CHECKING), "Published CHECKING"),
          
          // EBS CHECK
          // ASSERT_EBS_PRESSURE_NODE,
          // ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE,

          // ASSERT_SDC_OPEN_NODE,
          // TOGGLING_WATCHDOG_NODE,
          // CLOSE_SDC_NODE,
          // ASSERT_SDC_CLOSE_NODE,
          // STOP_TOGGLING_WATCHDOG_NODE,
          // ASSERT_SDC_OPEN_NODE,
          // TOGGLING_WATCHDOG_NODE,
          // OPEN_SDC_NODE,
          // ASSERT_SDC_OPEN_NODE,

          // UNBRAKE_ACT1_NODE,
          // ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,
          // BRAKE_ACT1_NODE,
          // UNBRAKE_ACT2_NODE,
          // ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE,

          ENABLE_MOTORS_NODE,

          WAIT_BRAKE_MOTOR_ENABLED,
          // UNBRAKE_ACT1_NODE,
          // ASSERT_NO_BRAKE_PRESSURE_NODE,
          // BRAKE_WITH_MAXON_MOTOR_NODE,
          // ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR,

          // CLOSE_SDC_NODE,
          // WAIT_TS_ACTIVE,

          // READY
          doActionNode([]{
            hal::send_current_state(AsState::READY);
            assi_manager::AssiManager::getInstance().ready();
          }, "Published READY and ASSI to READY"),
          WAIT_GO_SIGNAL,
          doActionNode([]{
            assi_manager::AssiManager::getInstance().driving();
          },
            "ASSI to DRIVING"
          ),
          PULL_CLUTCH_NODE,
          GEAR_FIRST_NODE,

          // DRIVING
          doActionNode(std::bind(&hal::send_current_state, AsState::DRIVING), "Published DRIVING"),
          WAIT_STOP_SIGNAL,

          terminalTrapNode(
            []{
              hal::send_current_state(AsState::FINISHED);
              open_sdc(); brake_act1(); brake_act2();
              assi_manager::AssiManager::getInstance().finished();
            }, "FINISHED State")

        },
        terminalTrapNode(
          []{
            hal::send_current_state(AsState::EMERGENCY);
            open_sdc(); brake_act1(); brake_act2();
            assi_manager::AssiManager::getInstance().emergency();
          }, "EMERGENCY State")
    ) {}
}