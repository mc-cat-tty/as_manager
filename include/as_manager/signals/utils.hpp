#pragma once

#include <functional>
// 0x45 0x4e 0x4f 0x49 0x4c 0x47 0x4f 0x43
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