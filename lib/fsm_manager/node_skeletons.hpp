/**
 * Collection of skeleton - template - functions that model
 * nodes of the FSM.
 */
#include "fsm_manager.hpp"

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
    std::function<bool()> predicate,
    std::chrono::milliseconds ms,
    std::string passedMsg,
    std::string failedMsg,
    std::string timeoutMsg
  ){
    static auto timer = Timer(Clock());
    timer.start(ms);

    if (predicate()) {
      std::cout << passedMsg << std::endl;
      timer.stop();
      return NodeFlowCtrl::NEXT;
    }
    
    if(timer.has_expired()) {
        std::cout << timeoutMsg << std::endl;
        timer.stop();
        throw std::exception();
    }

    std::cout << failedMsg << std::endl;

    return NodeFlowCtrl::CURRENT;
  };

  /**
   * @brief Skeleton function that waits until a predicate is met.
   * Can propagate exceptions if emergency state is triggered.
   */
  template <SafetyMonitoringSwitch doSafetyMonitoring = SafetyMonitoringSwitch::DISABLE>
  NodeFlowCtrl constexpr waitUntil(
    std::function<bool()> predicate,
    std::string successfulMsg,
    std::string failedMsg
  ) {
    if (predicate()) {
      std::cout << successfulMsg << std::endl;
      return NodeFlowCtrl::NEXT;  // Next
    }
    
    if constexpr (doSafetyMonitoring == SafetyMonitoringSwitch::ENABLE) {
      auto c = ContinousMonitoring();
      c.run();
      std::cout << "Monitoring" << std::endl;
    }
    
    std::cout << failedMsg << std::endl;
    return NodeFlowCtrl::CURRENT;
  }

  /**
   * @brief Skeletion function that performs and action and hands over to the next node.
   */
  NodeFlowCtrl doAction(
    std::function<void()> action,
    std::string msg
  ) {
    std::cout << msg << std::endl;
    action();
    return NodeFlowCtrl::NEXT;
  }

}