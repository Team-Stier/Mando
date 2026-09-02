# Disable Steering Runtime Fault Design

## Goal

Allow a steering target to remain commanded for any duration without raising
fault 4 solely because nonzero steering PWM has continued for a fixed time.

## Design

- Interpret `kSteeringMaxDriveMs == 0` as "continuous-runtime monitoring is
  disabled."
- Set the T870 configuration value to `0`.
- Keep fault 3 enabled: authorized nonzero PWM with less than 2 ADC of movement
  for 1000 ms still latches `FAULT_STEERING_STALL`.
- Keep steering sensor electrical checks, safe endpoints, approach limiting,
  target deadband, PWM limits, and all drive-motor protections unchanged.
- Keep the fault 4 enum and data fields for compatibility; a future nonzero
  configuration can enable the runtime limit again.

## Verification

- A compile-time behavior test must prove that a zero runtime limit never
  expires, including after more than the old 2500 ms window.
- Existing self-tests and both Uno RC and ROS builds must compile.
