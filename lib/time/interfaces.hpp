#pragma once

#include <time/common.hpp>

namespace temporal {
  struct IClock {
    virtual Tick get_time() = 0; 
  };
}