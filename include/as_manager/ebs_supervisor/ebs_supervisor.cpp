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
          doActionNode(std::bind(hal::send_current_state, AsState::OFF), "Published OFF"),

          INIT_PINS_NODE,
          START_CANBUS_NODE,
          
          WAIT_ORIN_ON_NODE,
          WAIT_ASMS_NODE,
          
          START_CANOPEN_NODE,
          
          WAIT_MISSION_NODE,
          START_CONTROL_NODE,
          doActionNode(std::bind(hal::send_current_state, AsState::CHECKING), "Published CHECKING"),
          
          // EBS CHECK
          safetyNodeDecorator(ASSERT_EBS_PRESSURE_NODE),
          safetyNodeDecorator(ASSERT_SUFFICIENT_BRAKE_PRESSURE_ALL_ACT_NODE),

          safetyNodeDecorator(STOP_TOGGLING_WATCHDOG_NODE),
          safetyNodeDecorator(OPEN_SDC_NODE),
          safetyNodeDecorator(ASSERT_SDC_OPEN_NODE),
          safetyNodeDecorator(TOGGLING_WATCHDOG_NODE),
          safetyNodeDecorator(CLOSE_SDC_NODE),
          safetyNodeDecorator(ASSERT_SDC_CLOSE_NODE),
          safetyNodeDecorator(STOP_TOGGLING_WATCHDOG_NODE),
          safetyNodeDecorator(ASSERT_SDC_OPEN_NODE),
          safetyNodeDecorator(TOGGLING_WATCHDOG_NODE),
          safetyNodeDecorator(OPEN_SDC_NODE),
          safetyNodeDecorator(ASSERT_SDC_OPEN_NODE),

          safetyNodeDecorator(UNBRAKE_ACT1_NODE),
          safetyNodeDecorator(ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE),
          safetyNodeDecorator(BRAKE_ACT1_NODE),
          safetyNodeDecorator(UNBRAKE_ACT2_NODE),
          safetyNodeDecorator(ASSERT_SUFFICIENT_BRAKE_PRESSURE_NODE),

          WAIT_CANOPEN_ON_NODE,
          ENABLE_MOTORS_NODE,
          WAIT_BRAKE_AND_CLUCTH_MOTORS_ENABLED_NODE,
          safetyNodeDecorator(UNBRAKE_ACT1_NODE),
          safetyNodeDecorator(ASSERT_NO_BRAKE_PRESSURE_NODE),
          safetyNodeDecorator(BRAKE_WITH_MAXON_MOTOR_NODE),
          safetyNodeDecorator(ASSERT_SUFFICIENT_BRAKE_PRESSURE_WITH_MAXON_MOTOR),

          safetyNodeDecorator(CLOSE_SDC_NODE),
          WAIT_TS_ACTIVE_NODE,

          // READY
          doActionNode([]{
            hal::send_current_state(AsState::READY);
            assi_manager::AssiManager::getInstance().ready();
          }, "Published READY and ASSI to READY"),
          WAIT_5_S_NODE,
          WAIT_GO_SIGNAL_OFF_NODE,  // Edge detector
          WAIT_GO_SIGNAL_ON_NODE,
          
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
