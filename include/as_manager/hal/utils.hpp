#pragma once

#include <cstdint>
#include <thread>
#include <functional>

namespace hal::utils {
  enum class Motors{
    Clutch=0x01,
    Stear=0x02,
    Brake=0x04,
    All=0x07
  };

  enum class ResBitVector{
    Go=0x01,
    Bag=0x02,
    Emergency=0x04
  };

  bool motorMask(Motors motor, uint8_t data);

  bool resMask(ResBitVector resState, uint8_t data);

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
