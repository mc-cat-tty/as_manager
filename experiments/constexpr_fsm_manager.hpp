#include <array>
#include <functional>
#include <utility>
#include <variant>
#include <initializer_list>
#include <exception>
#include <iostream>
#include <atomic>
#include <chrono>
#include <concepts>

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
  template <Node& ErrorNode, Node& ...Nodes>
  class Manager {
    private:
    std::size_t currentNodeIdx;
    std::size_t nodesNumber;
    std::array<Node, sizeof ...(Nodes)> nodes;
    Node errorNode;
    std::atomic_bool errorHandler;
    
    public:
    constexpr Manager(unsigned entryIdx = 0) :
      nodesNumber(sizeof ...(Nodes)), nodes{Nodes...}, errorNode(ErrorNode) {}
    
    void run() {
      auto currentNode = this->nodes.at(currentNodeIdx);
      auto retState = NodeFlowCtrl::CURRENT;

      // If emergency occurs, FSM gets trapped in emergency state
      if (this->errorHandler) {
        this->errorNode();
        return;
      }

      try {
        retState = currentNode();
      }
      catch (...) {
        this->errorHandler = true;
        std::cout << "Major order problem occurred" << std::endl;
        this->errorNode();
      }
      
      switch (retState) {
        case NodeFlowCtrl::CURRENT:
        break;
        
        case NodeFlowCtrl::NEXT:
        if (this->currentNodeIdx < this->nodesNumber-1)
            ++this->currentNodeIdx;
        break;

        default:
        std::unreachable();
      }
    }
    
    inline void operator() () { this->run(); }
  };


NodeFlowCtrl a() {}
NodeFlowCtrl b() {}
NodeFlowCtrl c() {}
NodeFlowCtrl e() {}

int main() {
    auto m = Manager<e, a, b, c>();
}