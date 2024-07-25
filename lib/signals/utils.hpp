#pragma once

#include <functional>

namespace signals::utils {
  template<typename cmpOp = std::less<float>>
  std::function<bool()> thresholdDecorator(
    std::function<float()> f,
    float threshold
  ) {
    return [f, threshold] -> bool {
      return cmpOp{}(f(), threshold);
    };
  }
};