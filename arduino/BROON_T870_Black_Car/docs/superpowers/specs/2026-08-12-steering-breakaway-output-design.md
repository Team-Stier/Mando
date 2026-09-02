# Steering Breakaway Output Design

## Problem

The steering controller currently produces PWM 40 through 80. Inside the
100-ADC endpoint approach band, `guardSteeringPwm()` scales that output all the
way toward zero. For example, PWM 40 at 30 ADC from an endpoint becomes PWM 12.
That output can be below the steering gearbox's breakaway requirement, so the
potentiometer stops changing and fault 3 latches after one second.

## Approved Control Values

- Steering minimum PWM: 60
- Steering maximum PWM: 140
- Steering proportional gain (`Kp`): 0.5
- Endpoint approach band: 100 ADC, unchanged
- Endpoint approach minimum nonzero PWM: 60
- Out-of-range inward recovery PWM: 60
- Steering deadband: 8 ADC, unchanged
- Safe endpoint ADC values: left 850 and right 170, unchanged
- Fault 3: require at least 2 ADC of progress within 1000 ms, unchanged
- Fault 4: disabled, unchanged

## Output Behavior

`computeSteeringPwm()` produces zero inside the target deadband and otherwise
produces a signed output between 60 and 140. `guardSteeringPwm()` may reduce a
large request inside the endpoint approach band, but a permitted nonzero output
must not fall below PWM 60. At or beyond an endpoint, motion farther outward is
still blocked immediately; only inward recovery is permitted.

This floor applies only after the requested direction has passed endpoint
validation. It does not override the target deadband, output authorization,
sensor fault handling, remote stop, or endpoint hard stop.

## Verification

- A regression test must reproduce the old PWM-12 endpoint result and require
  PWM 60 instead.
- Tests must preserve zero output in the deadband and outward blocking at both
  safe endpoints.
- The self-test, normal RC build, and ROS build must compile for Arduino Uno.
- Runtime behavior must be checked with the wheels lifted after upload; compile
  tests cannot prove the physical motor's speed or breakaway threshold.
