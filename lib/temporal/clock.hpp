#pragma once

#include <chrono>

namespace temporal {
  using Tick = std::chrono::milliseconds;

  struct Clock {
    static inline Tick get_time() {
      return std::chrono::duration_cast<Tick>(std::chrono::high_resolution_clock::now().time_since_epoch());
    }
  };
}
