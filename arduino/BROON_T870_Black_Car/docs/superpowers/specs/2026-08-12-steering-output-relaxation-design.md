# Steering Output Relaxation Design

## Goal

Prevent nuisance steering stall faults caused by the conservative initial PWM
and progress thresholds while retaining the measured mechanical end guards.

## Changes

- Raise steering minimum PWM from 25 to 40.
- Raise steering maximum PWM from 45 to 80.
- Count 2 ADC of potentiometer movement as progress instead of 4 ADC.
- Raise the no-progress fault window from 500 ms to 1000 ms.
- Raise maximum continuous steering drive time from 1500 ms to 2500 ms.
- Raise out-of-range recovery PWM from 25 to 40.
- Keep Kp 0.25, deadband 8 ADC, approach band 100 ADC, safe endpoints
  850/170, and all latched-fault behavior unchanged.

## Diagnostics and Verification

Human-readable status output will include the guarded and authorized steering
PWM so the next lifted-wheel test can distinguish weak output from an electrical
or mechanical failure. Compile-time self-test assertions pin the new tuning
values. The Uno self-test, default controller, and ROS controller must compile.

Physical verification is limited to a lifted-wheel test. Fault 3 remains a
latched fault and requires an Arduino reset after correcting its cause.
