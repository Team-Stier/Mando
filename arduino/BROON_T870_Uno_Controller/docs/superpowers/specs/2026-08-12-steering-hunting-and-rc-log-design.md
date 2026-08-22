# Steering hunting suppression and compact RC log design

## Scope

Change only steering hunting behavior and the human-readable RC diagnostic
line. Do not change RC calibration, steering calibration, steering maximum
PWM, drive behavior, ROS transport, or safety fault thresholds.

## Steering behavior

- Keep the existing normalized steering command and the 60..140 PWM range.
- Enter a settled state when the absolute target-versus-current ADC error is
  15 counts or less. Output zero while settled.
- Leave the settled state only when the absolute ADC error reaches 30 counts.
- When a nonzero request changes direction, output zero for one control update
  before allowing the opposite direction. This removes immediate torque
  reversal without reducing output far from the target.
- Immediate stop, endpoint guards, sensor checks, and steering watchdog rules
  remain authoritative.

## Human-readable RC log

Remove ROS-only target speed and ROS PI PWM fields plus historical min/max and
raw encoder-count noise from the human-readable line. Keep compact fields for
state, fault, mode, RC stop, steering/throttle pulses, requested/applied drive
PWM, steering target/current/applied PWM, measured speed, and RC read time.

## Transmitter-off observation

A receiver may continue emitting its stored failsafe pulse after transmitter
loss. A steering pulse near 1335..1340 us maps to roughly ADC 730 with the
unchanged 1100/1450/1800 us and 1000/600/200 ADC calibration. This change does
not alter receiver failsafe programming or infer transmitter presence from a
still-valid PWM pulse.

## Verification

- Self-tests cover settle entry, restart hysteresis, one-update reversal
  neutralization, and continued full response outside the settle region.
- Compile the self-test, RC build, and ROS build for Arduino Uno.
