#pragma once

#include <fsm_manager/skeletons.hpp>
#include <concepts>

namespace as::fsm {
  constexpr inline auto assertWithTimeoutNode(
    std::predicate auto predicate,
    std::chrono::milliseconds ms,
    std::string_view successfulMsg,
    std::string_view waitingMsg,
    std::string_view timeoutMsg
  ) {
    return [
      predicate,
      ms,
      successfulMsg,
      waitingMsg,
      timeoutMsg
    ] {
      return assertWithTimeout(predicate, ms, successfulMsg, waitingMsg, timeoutMsg);
    };
  }

  constexpr inline auto continousMonitoringAssertNode(
    std::invocable auto predicate,
    std::chrono::milliseconds ms,
    temporal::Timer& timer,
    std::string_view waitingMsg,
    std::string_view timeoutMsg
  ) {
    return [
      predicate,
      ms,
      timer,
      waitingMsg,
      timeoutMsg
    ] {
      return assertWithTimeout(predicate, ms, timer, waitingMsg, timeoutMsg);
    };
  }

  template <SafetyMonitoringSwitch doSafetyMonitoring = SafetyMonitoringSwitch::DISABLE>
  constexpr inline auto waitUntilNode(
    std::predicate auto predicate,
    std::string_view successfulMsg,
    std::string_view waitingMsg,
    std::invocable auto continousMonitoring
  ) {
    return [
      predicate,
      successfulMsg,
      waitingMsg,
      continousMonitoring
    ] {
      return waitUntil<doSafetyMonitoring>(predicate, successfulMsg, waitingMsg, continousMonitoring);
    };
  }

  constexpr inline auto doActionNode(
    std::invocable auto action,
    std::string_view msg
  ) {
    return [action, msg] {
      return doAction(action, msg);
    };
  }

  constexpr inline auto terminalTrapNode(
    std::invocable auto fn,
    std::string_view msg
  ) {
    return [fn, msg] {
      return terminalTrap(fn, msg);
    };
  }
}