#include <array>
#include <functional>
#include <utility>
#include <variant>
#include <initializer_list>
#include <iostream>

#include "as/exception.hpp"

namespace fsm {
  /**
  * @brief Each node can either transition to the next node or transition
  * to himself (self-loop). In case of an exception, a pre-defined non-accepting
  * state is reached.
  **/
  enum class NodeFlowCtrl {
    CURRENT,
    NEXT
  };

  template <std::size_t NodesNumber> class Manager;

  /// A node can be a function or a sub-manager (subset of independent nodes)
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
    
    public:
    Manager(std::initializer_list<Node> nodeList, Node emergencyNode, unsigned entryIdx = 0) :
      nodes(nodeList), emergencyNode(emergencyNode), currentNodeIdx(entryIdx) {}
    
    void run() {
      auto currentNode = this->nodeList.at(currentNodeIdx);
      auto retState = NodeFlowCtrl::CURRENT;

      try {
        retState = currentNode();
      }
      catch (as::EmergencyException e) {
        std::cout << e.what() << std::endl;
        this->emergencyNode();
      }
      catch (...) {
        std::cout << "Major order problem occurred" << std::endl;
        this->emergencyNode();
      }
      
      switch (retState) {
        case NodeFlowCtrl::CURRENT:
        break;
        
        case NodeFlowCtrl::NEXT:
        ++this->currentNodeIdx;
        break;

        default:
        std::unreachable();
      }
    }
    
    inline void operator() () { this->run(); }
  };


};
