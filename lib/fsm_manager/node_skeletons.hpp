/**
 * Collection of skeleton - template - functions that model
 * nodes of the FSM.
 */
#include "fsm_manager.hpp"

namespace as::fsm {
  /**
   * @brief Skeleton function that checks for a condition
   * to be true within a time window defined by ms.
   */
  template <std::predicate CheckFn>
  NodeFlowCtrl assertWithTimeout(
    std::chrono::milliseconds ms,
    std::string passedMsg,
    std::string failedMsg,
    std::string timeoutMsg
  ){
    static auto timer = Timer(Clock());
    timer.start(ms);

    if (CheckFn()) {
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


}