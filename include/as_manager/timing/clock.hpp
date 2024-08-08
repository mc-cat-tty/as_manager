#pragma once
#if STANDALONE

#include <chrono>

namespace timing {
  using namespace std::chrono;

  struct Clock {
      template <class duration>
      static inline duration get_time() {
          return duration_cast<duration>(steady_clock::now().time_since_epoch());
      }
  };
}

#else
#include <mmr_base/configuration.hpp>
#endif