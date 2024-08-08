#pragma once

#include <cstdint>
#include <functional>
#include <cmath>
#include <as_manager/timing/clock.hpp>
#include <as_manager/common/common_types.hpp>

namespace hal::utils {
  constexpr inline static int getBitIdx(enumerable auto m) {
    return static_cast<int>(log2(static_cast<unsigned>(m)));
  }

  bool mask(unsigned data, maskable auto mask);
  bool mask(unsigned data, unsigned mask);

  uint8_t motorsComposeBv(bool clutch, bool steer, bool brake);
  uint8_t resComposeBv(bool go, bool bag, bool emergency);

  /**
  @brief Synchronous blocking function that emulates a switch over
  CAN bus for BOSCH ECUs, generating a falling edge by spamming
  up value for N times with M inter-send microseconds, then down value
  with the same temporization.
  */
  void ecuButtonTrigger(
    std::function<void(bool)> sendFunction,
    std::chrono::milliseconds ms
  );
}
