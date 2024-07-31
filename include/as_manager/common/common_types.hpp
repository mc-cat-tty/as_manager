#pragma once

#include <unordered_map>
#include <string_view>
#include <concepts>

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
    Open=1,
    Closed=0
  };

  enum class ActuatorState {
    Braking,
    Unbrake
  };

  // out
  enum class AssiState{
    On,
    Strobe,
    Off
  };

  enum class BuzzerState{
    On,
    Off
  };

  enum class ResState {
    Idle,
    Operational,
    Error
  };
};

namespace as {
  //topic: /ebs/status
  enum class EbsSupervisorState {
    OFF,
    CHECKING,
    READY,
    DRIVING,
    FINISHED,
    EMERGENCY
  };

  const inline std::unordered_map<EbsSupervisorState, std::string_view> EbsSupervisorStateLookup {
    { as::EbsSupervisorState::OFF, "OFF" },
    { as::EbsSupervisorState::CHECKING, "CHECKING"},
    { as::EbsSupervisorState::READY, "READY" },
    { as::EbsSupervisorState::DRIVING, "DRIVING" },
    { as::EbsSupervisorState::FINISHED, "FINISHED" },
    { as::EbsSupervisorState::EMERGENCY, "EMERGENCY" }
  };
}
