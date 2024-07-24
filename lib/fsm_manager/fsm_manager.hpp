#include <array>
#include <functional>
#include <utility>
#include <variant>
#include <initializer_list>

namespace fsm {
  /**
  * @brief Each node can either transition to the next node or transition
  * to himself (self-loop). In case of an error, a pre-defined non-accepting
  * state is reached.
  **/
  enum class NodeState {
    CURRENT = 0,
    NEXT,
    ERROR
  };

  // A node can be a function or a sub-manaer (nodes subset)
  using Node = std::variant<std::function<NodeState(int, float)>>;

  template <std::initializer_list<Node> nodes>
  struct Manager {
    std::array<Node, std::size()> nodeList;
    FSMNode error, finished;
    unsigned idx;
    
    FSMManager(std::array<FSMNode, 10> nodeList, FSMNode error, FSMNode finished, unsigned entry = 0) :
      nodeList(nodeList), error(error), finished(finished), idx(entry) {}
    
    void run() {
      static FSMNode currentNode = this->nodeList.at(idx);
      auto ret = currentNode();
      if (ret == FSMState::CURRENT) ;
      else if (ret == FSMState::NEXT) ++idx;
      else if (ret == FSMState::ERROR) error();
      else if (ret == FSMState::FINISHED) finished();
      else std::unreachable();
    }
  };
};
