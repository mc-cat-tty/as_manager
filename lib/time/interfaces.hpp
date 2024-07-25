#pragma once

#include <chrono>

namespace temporal {

  using Tick = std::chrono::milliseconds;
  struct IClock {
    virtual Tick get_time() = 0; 
  };
}