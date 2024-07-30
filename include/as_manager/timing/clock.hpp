#pragma once

#include <chrono>

namespace timing {
  using namespace std::chrono;

  using Tick = milliseconds;

  struct Clock {
    static inline Tick get_time() {
      return duration_cast<Tick>(high_resolution_clock::now().time_since_epoch());
    }
  };
}
