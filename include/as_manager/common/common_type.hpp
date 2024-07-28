#pragma once

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
}
