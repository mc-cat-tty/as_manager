#pragma once

#include <unordered_map>
#include <string_view>
#include <concepts>

#if !STANDALONE
#include <mmr_base/configuration.hpp>
#endif

namespace hal {
  template<typename T> concept enumerable = std::is_enum_v<T>;
  template <typename T>
  concept maskable = requires(T x) {
    { x } -> std::convertible_to<unsigned>;
  };

  template<enumerable T>
  unsigned operator|(const T &first, const T &second) {
    return static_cast<unsigned>(first) | static_cast<unsigned>(second);
  }

  template<enumerable T>
  unsigned operator|(const unsigned &first, const T &second) {
    return first | static_cast<unsigned>(second);
  }

  enum class MaxonMotors {
    CLUTCH = 0x01,
    STEER = 0x02,
    BRAKE = 0x04
  };

  enum class Res {
    GO = 0x01,
    BAG = 0x02,
    EMERGENCY = 0x04
  };

  enum class SdcState {
    OPEN=1,
    CLOSE=0
  };

  enum class ActuatorState {
    BRAKING,
    UNBRAKING
  };

  // out
  enum class AssiState{
    ON,
    STROBE,
    OFF
  };

  enum class BuzzerState{
    ON,
    OFF
  };

  enum class ResState {
    IDLE,
    OPERATIONAL,
    ERROR
  };
};

namespace as {
  #if STANDALONE
  enum class AsState {
    OFF,
    CHECKING,
    READY,
    DRIVING,
    FINISHED,
    EMERGENCY
  };

  const inline std::unordered_map<EbsSupervisorState, std::string_view> AsStateStringLookup {
    { as::AsState::OFF, "OFF" },
    { as::AsState::CHECKING, "CHECKING"},
    { as::AsState::READY, "READY" },
    { as::AsState::DRIVING, "DRIVING" },
    { as::AsState::FINISHED, "FINISHED" },
    { as::AsState::EMERGENCY, "EMERGENCY" }
  };
  #else
  using AsState = AS::STATE;
  #endif
}
