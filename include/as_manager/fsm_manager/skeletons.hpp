/**
 * Collection of skeleton - template - functions that model
 * nodes of the FSM.
 */
#pragma once

#include <string_view>
#include <concepts>
#include <as_manager/fsm_manager/fsm_manager.hpp>
#include <as_manager/timing/timer.hpp>
#include <as_manager/timing/clock.hpp>
#include <as_manager/as/exception.hpp>

namespace as::fsm {
  enum class SafetyMonitoringSwitch {
    DISABLE = 0,
    ENABLE
  };

  /**
   * @brief Skeleton function that checks for a condition
   * to be true within a time window defined by ms.
   */
  NodeFlowCtrl assertWithTimeout(
    std::predicate auto predicate,
    std::chrono::milliseconds ms,
    std::string_view successfulMsg,
    std::string_view waitingMsg,
    std::string_view timeoutMsg
  ){
    static auto timer = timing::TimerAsync();
    static auto isWaitMsgLogged = false;
    timer.start(ms);

    if (predicate()) {
      std::cout << successfulMsg << std::endl;
      timer.stop();
      isWaitMsgLogged = false;
      return NodeFlowCtrl::NEXT;
    }
    
    if(timer.has_expired()) {
        std::cout << timeoutMsg << std::endl;
        timer.stop();
        isWaitMsgLogged = false;
        throw EmergencyException();
    }

    if (not isWaitMsgLogged) {
      std::cout << waitingMsg << std::endl;
      isWaitMsgLogged = true;
    }

    return NodeFlowCtrl::CURRENT;
  };

  /**
   * @brief Skeleton function for continous monitoring
   */
  void continousMonitoringAssert(
    std::predicate auto predicate,
    std::chrono::milliseconds ms,
    timing::TimerAsync& timer,
    std::string_view waitingMsg,
    std::string_view timeoutMsg
  ){
    timer.start(ms);

    if (predicate()) {
      timer.stop();
    }
    
    if(timer.has_expired()) {
        std::cout << timeoutMsg << std::endl;
        timer.stop();
        throw EmergencyException();
    }

    std::cout << waitingMsg << std::endl;
  };

  /**
   * @brief Skeleton function that waits until a predicate is met.
   * Can propagate exceptions if emergency state is triggered.
   */
  template <SafetyMonitoringSwitch doSafetyMonitoring = SafetyMonitoringSwitch::DISABLE>
  NodeFlowCtrl waitUntil(
    std::predicate auto predicate,
    std::string_view successfulMsg,
    std::string_view waitingMsg,
    std::invocable auto continousMonitoring = []{}
  ) {
    static auto isWaitMsgLogged = false;

    if (predicate()) {
      std::cout << successfulMsg << std::endl;
      isWaitMsgLogged = false;
      return NodeFlowCtrl::NEXT;  // Next
    }
    
    if constexpr (doSafetyMonitoring == SafetyMonitoringSwitch::ENABLE) {
      continousMonitoring();
      std::cout << "Monitoring" << std::endl;
    }
    
    if (not isWaitMsgLogged) {
      std::cout << waitingMsg << std::endl;
      isWaitMsgLogged = true;
    }

    return NodeFlowCtrl::CURRENT;
  }

  /**
   * @brief Skeleton function that performs and action and hands control over to the next node.
   */
  constexpr NodeFlowCtrl doAction(
    std::invocable auto action,
    std::string_view msg
  ) {
    std::cout << msg << std::endl;
    action();
    return NodeFlowCtrl::NEXT;
  }

  /**
   * @brief Skeleton that calls function indefinitely and traps the execution flow of the FSM,
   * by returning NodeFlowCtrl::NEXT at each iteration.
   */
  NodeFlowCtrl terminalTrap(
    std::function<void()> fn,
    std::string_view msg
  ) {
    fn();
    std::cout << msg << std::endl;
    return NodeFlowCtrl::CURRENT;
  }
}