# Steering Breakaway Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent endpoint approach scaling from reducing steering output below breakaway PWM while increasing normal steering authority conservatively.

**Architecture:** Keep position control and endpoint blocking unchanged. Extend `guardSteeringPwm()` with an explicit approach minimum so allowed nonzero output is clamped to PWM 60 after scaling; configure P control for PWM 60 through 140 at Kp 0.5.

**Tech Stack:** Arduino AVR C++, Arduino Uno, `arduino-cli`

## Global Constraints

- Steering minimum PWM is 60 and maximum PWM is 140.
- Steering Kp is 0.5.
- Endpoint approach band remains 100 ADC and allowed nonzero approach output has a floor of PWM 60.
- Inward recovery PWM is 60.
- Safe endpoints remain left ADC 850 and right ADC 170.
- Fault 3 remains 2 ADC of required progress within 1000 ms.
- Fault 4 remains disabled.
- RC calibration, pin mapping, drive control, and output lock values do not change.

---

### Task 1: Preserve breakaway output through endpoint guarding

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`

**Interfaces:**
- Consumes: signed requested PWM, current ADC, endpoint calibration, approach band, maximum PWM, approach minimum PWM, and recovery PWM.
- Produces: `guardSteeringPwm(..., uint8_t maximumPwm, uint8_t minimumPwm, uint8_t recoveryPwm)` returning an allowed signed PWM that is either zero or has magnitude at least `minimumPwm` while inside the safe range.

- [x] **Step 1: Change the self-test to require PWM 60 through 140, Kp 0.5, recovery PWM 60, and a PWM-60 result for a right endpoint approach that previously produced PWM 12.**
- [x] **Step 2: Compile the self-test and verify failure against the old function signature and configuration.**
- [x] **Step 3: Add the approach-minimum argument and clamp scaled nonzero endpoint output to that magnitude without bypassing endpoint blocking.**
- [x] **Step 4: Set the steering configuration to minimum 60, maximum 140, recovery 60, and Kp 0.5; pass the minimum into the endpoint guard.**
- [x] **Step 5: Update operator and calibration documentation with the new values and endpoint-floor behavior.**
- [x] **Step 6: Compile the self-test, standard RC sketch, and ROS sketch for Arduino Uno and audit unchanged safety values.**
