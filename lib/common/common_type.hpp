#pragma once

namespace as {

enum class SdcState {
    Open,
    Closed
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

//topic: /ebs/status
typedef enum EbsSupervisorState {
  OFF,
  CHECKING,
  READY,
  DRIVING,
  FINISHED,
  EMERGENCY
} EbsSupervisorState;


};
