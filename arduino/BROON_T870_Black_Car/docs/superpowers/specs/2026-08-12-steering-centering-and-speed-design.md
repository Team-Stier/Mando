# Steering centering and speed adjustment design

## Scope

Change only steering settle/restart thresholds and the symmetric maximum
steering PWM. Do not change steering calibration, RC calibration, minimum PWM,
direction-reversal neutralization, endpoint guards, or fault behavior.

## Settings

- Settle when target-versus-current error is 5 ADC counts or less.
- After settling, restart only when error reaches 18 ADC counts.
- Increase symmetric maximum steering PWM from 140 to 170.
- Keep minimum PWM at 60 and recovery PWM at 60.

The 595..605 settle interval around center ADC 600 limits the ideal
opposite-approach final-position difference to 10 ADC counts. The larger
maximum PWM increases motion speed mainly when the target is far away; the
existing normalized proportional curve keeps near-target output close to the
minimum PWM.

## Verification

Update configuration assertions and normalized full-error PWM expectations,
then compile the self-test, default RC sketch, and ROS sketch for Arduino Uno.
