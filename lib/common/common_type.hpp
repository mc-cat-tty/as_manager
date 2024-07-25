#ifndef LIB_COMMON_COMMON_TYPE_HPP
#define LIB_COMMON_COMMON_TYPE_HPP

namespace ebs_supervisor {

enum class SdcState {
    Open,
    Closed
};

enum class ActuatorState {
    Braking,
    Released
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

#endif
