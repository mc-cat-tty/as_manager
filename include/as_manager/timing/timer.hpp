#pragma once

#include <as_manager/timing/clock.hpp>

namespace timing {
  class TimerAsync {
  public:
    TimerAsync();
    bool has_expired() const;
    void start(timing::Tick duration);
    void stop();

  private:
    bool is_started;
    timing::Tick endtime;
    timing::Tick duration;
  };
};