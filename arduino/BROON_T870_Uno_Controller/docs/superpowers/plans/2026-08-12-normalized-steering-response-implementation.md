# Normalized Steering Response Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce symmetric steering PWM from asymmetric ADC calibration and log RC snapshot latency.

**Architecture:** Convert target and current ADC to a piecewise normalized steering coordinate before calculating signed PWM. Preserve endpoint and fault guards, and instrument the existing sequential `pulseIn()` snapshot rather than replacing it.

**Tech Stack:** Arduino AVR C++, Arduino Uno, `arduino-cli`

## Global Constraints

- Left, center, and right safe ADC values remain 850, 640, and 170.
- Steering PWM remains 60 through 140.
- Steering deadband remains 8 ADC.
- Endpoint approach band remains 100 ADC with PWM-60 floor.
- Fault 3 remains 2 ADC within 1000 ms; fault 4 remains disabled.
- RC continues to use `pulseIn()` on A0, A1, and A2.
- Pin mapping, RC calibration, drive control, and output locks do not change.

---

### Task 1: Normalize steering position error

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Modify: `BROON_T870_Uno_Controller.ino`

**Interfaces:**
- Produces: `steeringPositionPermille(adc, leftAdc, centerAdc, rightAdc)` in the range -1000 through +1000.
- Produces: `computeNormalizedSteeringPwm(targetAdc, currentAdc, leftAdc, centerAdc, rightAdc, deadbandAdc, minimumPwm, maximumPwm)`.

- [x] **Step 1: Add failing behavior tests for symmetric half and full left/right PWM, sign, and deadband.**
- [x] **Step 2: Compile the self-test and confirm failure because the normalized API does not exist.**
- [x] **Step 3: Implement the normalized coordinate and PWM functions and use them in the main steering path.**
- [x] **Step 4: Remove the obsolete raw-ADC Kp configuration and validation.**

### Task 2: Measure RC read latency and verify builds

**Files:**
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`

**Interfaces:**
- Produces: human status field `rc_read_us=<elapsed microseconds>`.

- [x] **Step 1: Measure elapsed microseconds around `rcInput.snapshot()` and add it to the human status log.**
- [x] **Step 2: Document normalized steering and interpretation of `rc_read_us` versus `steer_pwm`.**
- [x] **Step 3: Compile the self-test, RC sketch, and ROS sketch for Arduino Uno and audit all unchanged constraints.**
