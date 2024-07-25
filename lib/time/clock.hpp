#pragma once

#include <time/interfaces.hpp>

namespace temporal {

  struct Clock {
    static inline Tick get_time() {
      return std::chrono::duration_cast<Tick>(std::chrono::high_resolution_clock::now().time_since_epoch());
    }
  };
}
