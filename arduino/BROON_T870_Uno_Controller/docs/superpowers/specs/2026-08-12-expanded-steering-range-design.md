# Expanded Steering Range Design

## Goal

Increase the RC steering target range because the previous left limit of ADC
850 produced insufficient physical steering angle.

## Approved Calibration

- Left mechanical endpoint: 999, unchanged
- Left commanded endpoint: 980
- Straight-ahead center: 640
- Right commanded endpoint: 280
- Right mechanical endpoint: 21, unchanged
- ADC decreases toward the right, unchanged

This leaves 19 ADC between the commanded and measured mechanical left ends and
259 ADC on the right. RC steering maps 1100/1450/1800 microseconds to
980/640/280 ADC. ROS steering uses the same endpoints through its existing
piecewise degree mapping.

## Unchanged Protections

- Normalized steering PWM remains 60 through 140.
- The 8-ADC target deadband, 100-ADC endpoint approach band, PWM-60 approach
  floor, sensor checks, and fault 3 remain active.
- Fault 4 remains disabled.
- Motion farther outward is blocked at ADC 980 on the left and ADC 280 on the
  right.

## Verification

- Tests require the RC endpoints and midpoints 980/810/640/460/280.
- Normalized steering tests require +1000/0/-1000 at 980/640/280 and symmetric
  PWM at the endpoints and midpoints.
- Endpoint guard tests require outward blocking and inward recovery at the new
  limits.
- Self-test, RC, and ROS sketches compile for Arduino Uno.
