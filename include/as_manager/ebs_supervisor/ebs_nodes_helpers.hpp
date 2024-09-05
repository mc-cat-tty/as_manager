#pragma once
#include <chrono>
#include <as_manager/common/common_types.hpp>
#include <as_manager/fsm_manager/nodes_factory.hpp>
#include <string>

namespace as::ebs_supervisor {
  template<SafetyMonitoringSwitch monitorSwitch = SafetyMonitoringSwitch::DISABLE>
  auto sleepNode(std::chrono::milliseconds ms) {
    return waitUntilNode<monitorSwitch>(
      [ms] {
        static timing::TimerAsync timer;
        timer.start(ms);

        if(timer.has_expired()){
          timer.stop();
          return true;
        }
        else {
          return false;
        } 
      }, 
      std::to_string(ms.count()) + "ms expired",
      "Waiting " + std::to_string(ms.count()) + "ms",
      [] { EbsContinousMonitoring::getInstance().continuousMonitoring(); }
    );
  }

  constexpr auto safetyNodeDecorator(std::invocable auto node) {
    return [node]() {
      if (not Parameters::getInstance().safetyFeatures) return NodeFlowCtrl::NEXT;
      return node();
    };
  }
}