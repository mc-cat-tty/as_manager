# Default Pin States
Safe State:
  - SDC CTRL OPEN
  - Watchdog OFF
  - Act1 BRAKING
  - Act2 BRAKING
  - State OFF

# EBS Supervisor States
## OFF
Wait for autonomous mission (all missions apart manual, debug) and ASMS ON
Enter CHECKING

## CHECKING
### Pre-check
Assert sufficient EBS storage pressure
Assert sufficient brake pressure

### Check SDC CTRL pin and watchdog
Assert SDC is open

Start toggling watchdog
Set SDC CTRL to CLOSED
Assert SDC is closed

Stop toggling watchdog
Assert SDC is open

Start toggling watchdog
Assert SDC is closed

Set SDC CTRL OPEN
Assert SDC is open

### Actuators check
Unbrake ACT1
Assert sufficient brake pressure

Brake ACT1
Unbrake ACT2
Assert sufficient brake pressure

### Check service brake (Maxon Motor)
Wait until /can/brake_motor [reliable, volatile, spammed] is enabled
Unbrake ACT1
Assert no brake pressure
Brake with service motor
Assert brake pressure

### Enable TS activation
Set SDC CTRL to CLOSED
Wait until TS active
Goto READY

## READY
Perform continuous monitoring while waiting RES GO signal
Pull Clutch
Insert Gear 1
Goto DRIVING

## DRIVING
Spin continuous monitoring and wait for STOP MESSAGE -> goto FINISHED

## FINISHED
All good
Safe State

## EMERGENCY
Safe State

## States Recap
States:
- Off
- Checking -> canopen
- Ready
- Driving
- Emergency
- Finished

# Continuous Monitoring
  - not RES Emergency
  - [not RES Error]
  - SDC Closed
  - EBS Tank Pressure
  - All motors alive

# ASSI Manager
States vs ASSI:
  - OFF & CHECKING - OFF
  - READY - STABLE Y
  - DRIVING - FLASHING Y
  - EMERGENCY - FLASHING B + BUZZER (1Hz + 50% duty)
  - FINISHED - STABLE B