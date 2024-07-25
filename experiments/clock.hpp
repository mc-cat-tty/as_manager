#include <array>
#include <functional>
#include <utility>
#include <variant>
#include <initializer_list>
#include <exception>
#include <iostream>
#include <atomic>
#include <chrono>

using Tick_t = std::chrono::milliseconds;

class Clock {
    public:
        Clock() {};
        Tick_t get_time();  
};

Tick_t Clock::get_time() {
    return std::chrono::duration_cast<Tick_t>(std::chrono::high_resolution_clock::now().time_since_epoch());
}

class Timer {
public:
    Timer(Clock& clock);

    bool has_expired() const;

    void start(Tick_t duration);

    void stop();

private:
    bool is_started;

    Tick_t endtime_;
    Tick_t duration;
    Clock& clock;
};


enum class NodeFlowCtrl {
  CURRENT,
  NEXT
};

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
}

/*
Node Types:
  - assert with timeout
  - action node
  - wait until
  - continuous monitoring node (spin continous monitoring + wait until)

Singal Types:
  - filtered
    - + threshold -> decorator pattern
    - raw
  - unfiltered

Signal(SourceI source) -> ... -> ... -> ... -> Final
*/

int main() {
}