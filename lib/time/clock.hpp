#pragma once

#include <time/common.hpp>

namespace temporal {
  using namespace std::chrono;

  struct Clock {
    static inline Tick get_time() {
      return duration_cast<Tick>(high_resolution_clock::now().time_since_epoch());
    }
  };
}
