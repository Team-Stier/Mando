# Aggressive steering response and endpoint hold design

## Scope

Increase commanded steering range and make normalized steering output reach
the existing maximum PWM much earlier. Add endpoint-specific hysteresis to
stop the observed -60/0/+60 chatter without reducing the requested endpoints.

## Calibration and response

- Steering calibration: left endpoint 1020, center 600, right endpoint 180.
- Keep minimum/recovery PWM 60 and maximum PWM 170.
- Reach PWM 170 at approximately 140 ADC counts of position error. With the
  symmetric 420-ADC half ranges, this is 333 normalized permille.
- Preserve the existing one-update zero output before direction reversal.

## Endpoint hold

- Treat targets within 5 ADC counts of either endpoint as maximum-steering
  targets.
- At a maximum-steering target, settle at absolute error <=25 ADC and remain
  settled until error reaches 60 ADC. The expanded 1020 target makes the
  captured 973 feedback an error of 47 ADC, so a 40-ADC restart would still
  re-enable the observed chatter.
- At all other targets, keep the general 5/18 ADC settle/restart hysteresis.
- Existing electrical sensor checks, endpoint guard, watchdog, and faults stay
  authoritative.

This directly addresses the captured left-end log: the target remained 1000
while feedback jumped between approximately 973 and 1018 and authorized PWM
alternated -60/0/+60. Endpoint hold ignores that bounce once the mechanism is
within the endpoint arrival band.

## Verification

Self-tests cover the 1020/600/180 endpoints, early PWM saturation, endpoint
target detection, and endpoint hysteresis. Compile self-test, default RC, and
ROS sketches for Arduino Uno.
