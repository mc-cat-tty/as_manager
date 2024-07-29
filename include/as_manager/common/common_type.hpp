#pragma once

#include <unordered_map>

namespace hal {
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

enum class ResSignal{
    Go,
    Bag,
    Emergency
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
