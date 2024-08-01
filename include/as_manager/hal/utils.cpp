#include "utils.hpp"
#include <thread>

namespace hal::utils {
  bool mask(unsigned data, unsigned mask) {
    return (data & mask) == mask;
  }

  bool mask(unsigned data, maskable auto mask) {
    return mask(data, static_cast<unsigned>(mask));
  }

  uint8_t resComposeBv(bool go, bool bag, bool emergency) {
    return (go << getBitIdx(Res::GO))
      | (bag << getBitIdx(Res::BAG))
      | (emergency << getBitIdx(Res::EMERGENCY));
  }

  uint8_t motorsComposeBv(bool clutch, bool steer, bool brake) {
    return (clutch << getBitIdx(MaxonMotors::CLUTCH))
      | (steer << getBitIdx(MaxonMotors::STEER))
      | (brake << getBitIdx(MaxonMotors::BRAKE));
  }

  void ecuButtonTrigger(
    std::function<void(bool)> sendFunction,
    timing::Tick ms
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