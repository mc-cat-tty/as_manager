#pragma once

#include <array>
#include <functional>
#include <utility>
#include <variant>
#include <initializer_list>
#include <iostream>
#include <atomic>

#include <as/exception.hpp>


namespace as::fsm {

  /**
  * @brief Each node can either transition to the next node or transition
  * to himself (self-loop). In case of an exception, a pre-defined non-accepting
  * error state is reached.
  **/
  enum class NodeFlowCtrl {
    CURRENT,
    NEXT
  };

  // A node can be a function or a sub-manager (subset of independent nodes)
  using Node = std::function<NodeFlowCtrl()>;

  /**
   * @brief fsm::Manager keeps a list of sequential nodes and a emergency node.
   * The emergency node is supposed to handle all the actions for a fail-safe stop.
   * The other states are executed sequentially; however, each state has the opportunity
   * to decide whether to stay on current state or transition to the next one.
  */
  template <std::size_t NodesNumber>
  class Manager {
    private:
    std::size_t currentNodeIdx;
    std::array<Node, NodesNumber> nodes;
    Node emergencyNode;
    std::atomic_bool errorHandlerTriggered = false;
    
    public:
    Manager(std::array<Node, NodesNumber> nodeList, Node emergencyNode, unsigned entryIdx = 0) :
      nodes(nodeList), emergencyNode(emergencyNode), currentNodeIdx(entryIdx) {}
    
    void run() {
      auto currentNode = this->nodes.at(currentNodeIdx);
      auto retState = NodeFlowCtrl::CURRENT;

      // If emergency occurs, FSM gets trapped in emergency state
      if (this->errorHandlerTriggered) {
        //this->emergencyNode();
        return;
      }

      try {
        retState = currentNode();
      }
      catch (as::EmergencyException e) {
        this->errorHandlerTriggered = true;
        std::cout << e.what() << std::endl;
        this->emergencyNode();
      }
      catch (...) {
        this->errorHandlerTriggered = true;
        std::cout << "Major order problem occurred" << std::endl;
        this->emergencyNode();
      }
      
      switch (retState) {
        case NodeFlowCtrl::CURRENT:
        break;
        
        case NodeFlowCtrl::NEXT:
        if (this->currentNodeIdx < NodesNumber-1) ++this->currentNodeIdx;
        break;

        default:
        std::unreachable();
      }
    }
    
    inline void operator() () { this->run(); }
  };

};