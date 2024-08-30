#pragma once

#include <as_manager/timing/clock.hpp>
#include <chrono>

namespace timing {
  class TimerAsync {
  public:
    TimerAsync();
    bool has_expired() const;
    void start(std::chrono::milliseconds duration);
    void restart(std::chrono::milliseconds duration);
    void restart();
    void stop();

  private:
    bool is_started;
    std::chrono::milliseconds endtime;
    std::chrono::milliseconds duration;
  };
};