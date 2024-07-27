#pragma once

#include <cassert>

namespace signals::utils {
  template <typename T>
  class LowPassFilter {
      private:
          float alpha;
          float filtered_value;

      public:
          inline float get_value() { return this->filtered_value; };
          inline float get_alpha() { return this->alpha; };

          inline LowPassFilter(float alpha) : alpha(alpha), filtered_value(0.0f) {
            assert(alpha >= 0.0f && alpha <= 1.0f);
          };

          inline void update(T val) {
            this->filtered_value = (1-alpha) * filtered_value + alpha * val;
          };
  };
}

