#include "utils.hpp"

namespace hal::utils {
  bool motorMask(Motors motor, uint8_t data){
    auto motorBitvector = static_cast<uint8_t>(motor);
    return (data & motorBitvector) == motorBitvector;
  }

  bool resMask(ResBitVector resState, uint8_t data){
    auto resBitVector= static_cast<uint8_t>(resState);
    return (data & resBitVector) == resBitVector;
  }

  void ecuButtonTrigger(
    std::function<void(bool)> sendFunction,
    std::chrono::milliseconds ms
  ) {
    using namespace std::this_thread;

    for (int i=0; i<5; i++) {
      sendFunction(true);
      sleep_for(ms);
    }
    for (int i=0; i<5; i++) {
      sendFunction(false);
      sleep_for(ms);
    }
  }
}