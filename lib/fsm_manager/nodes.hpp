#pragma once

#include <fsm_manager/skeletons.hpp>

namespace as::fsm {
  inline auto assertWithTimeoutNode(
    std::function<bool()> predicate,
    std::chrono::milliseconds ms,
    std::string successfulMsg,
    std::string waitingMsg,
    std::string timeoutMsg
  ) {
    return std::bind(assertWithTimeout, predicate, ms, successfulMsg, waitingMsg, timeoutMsg);
  }

    inline auto continousMonitoringAssertNode(
    std::function<bool()> predicate,
    std::chrono::milliseconds ms,
    temporal::Timer& timer,
    std::string waitingMsg,
    std::string timeoutMsg
  ) {
    return std::bind(assertWithTimeout, predicate, ms, timer, waitingMsg, timeoutMsg);
  }

  template <SafetyMonitoringSwitch doSafetyMonitoring = SafetyMonitoringSwitch::DISABLE>
  inline auto waitUntilNode(
    std::function<bool()> predicate,
    std::string successfulMsg,
    std::string waitingMsg,
    std::function<void()> continousMonitoring = []{}
  ) {
    return std::bind(waitUntil<doSafetyMonitoring>, predicate, successfulMsg, waitingMsg, continousMonitoring);
  }

  inline auto doActionNode(
    std::function<void()> action,
    std::string msg
  ) {
    return std::bind(doAction, action, msg);
  }

  inline auto terminalTrapNode(
    std::function<void()> fn,
    std::string msg
  ) {
    return std::bind(terminalTrap, fn, msg);
  }
}