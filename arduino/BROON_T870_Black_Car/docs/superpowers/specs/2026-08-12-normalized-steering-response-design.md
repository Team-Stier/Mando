# Normalized Steering Response Design

## Goal

Make steering response symmetric despite the asymmetric potentiometer spans,
and expose RC pulse-read latency without changing the proven `pulseIn()` input
method.

## Steering Control

The calibrated positions are left ADC 850, center ADC 640, and right ADC 170.
They map piecewise to normalized positions `+1000`, `0`, and `-1000`.
Target and current ADC values are converted independently; their normalized
position error determines PWM.

- Raw target error within 8 ADC still produces PWM 0.
- Any larger error produces at least PWM 60.
- A normalized error of 1000 or more produces PWM 140.
- Intermediate errors interpolate linearly between PWM 60 and PWM 140.
- Logical PWM remains negative toward left and positive toward right, matching
  the current motor-direction convention.
- Endpoint guarding, PWM-60 approach floor, ADC electrical checks, fault 3,
  and disabled fault 4 remain unchanged.

This makes center-to-left and center-to-right full commands both request PWM
140. It does not change the safe endpoints or claim that PWM 140 is physically
fast enough; that must be determined on the lifted vehicle.

## RC Latency Diagnostic

Keep sequential `pulseIn()` reads because they are the receiver-compatible
method confirmed on this vehicle. Measure the elapsed microseconds around each
three-channel snapshot and print it as `rc_read_us` in human-readable status.
This separates input latency from physical steering slew rate.

## Verification

- Self-tests require equal magnitude PWM for equal normalized left/right
  positions, full PWM 140 at both endpoints, PWM 0 inside the ADC deadband, and
  correct sign.
- Existing endpoint guard tests must continue to pass.
- Standard RC and ROS builds must compile for Arduino Uno.
- Physical testing must compare `steer_pwm` with ADC movement; PWM 140 with slow
  ADC movement indicates that the remaining limit is motor power or mechanics.
