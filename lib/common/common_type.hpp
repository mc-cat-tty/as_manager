#pragma once

namespace hal {
enum class SdcState {
    Open,
    Closed,
    Unknown
};

enum class ActuatorState {
    Braking,
    Unbrake
};

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
