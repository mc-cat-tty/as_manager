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
  * error state is reached.
  **/
  enum class NodeFlowCtrl {
    CURRENT,
    NEXT
  };

  template <std::size_t NodesNumber> class Manager;

  // A node can be a function or a sub-manager (subset of independent nodes)
  using Node = std::function<NodeFlowCtrl()>;

  template <std::size_t NodesNumber>
  class Manager {
    private:
    std::size_t currentNodeIdx;
    std::array<Node, NodesNumber> nodes;
    Node errorNode;
    
    public:
    Manager(std::initializer_list<Node> nodeList, Node errorNode, unsigned entryIdx = 0) :
      nodes(nodeList), errorNode(errorNode), currentNodeIdx(entryIdx) {}
    
    void run() {
      auto currentNode = this->nodeList.at(currentNodeIdx);
      auto retState = NodeFlowCtrl::CURRENT;

      try {
        retState = currentNode();
      }
      catch (as::EmergencyException e) {
        std::cout << e.what() << std::endl;
        this->errorNode();
      }
      catch (...) {
        std::cout << "Major order problem occurred" << std::endl;
        this->errorNode();
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
