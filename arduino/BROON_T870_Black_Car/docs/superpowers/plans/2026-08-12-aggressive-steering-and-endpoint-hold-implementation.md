# Aggressive Steering and Endpoint Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reach PWM 170 by roughly 140 ADC error while holding maximum steering without -60/0/+60 chatter.

**Architecture:** Extend the normalized PWM calculator with a configurable full-output error measured in normalized permille. Select general or endpoint-specific stabilizer hysteresis in the main sketch using a small pure endpoint-target predicate.

**Tech Stack:** Arduino AVR C++11, Arduino Uno, arduino-cli

## Global Constraints

- Steering range is 1020/600/180.
- Full PWM error is 333 normalized permille.
- General settle/restart remains 5/18 ADC.
- Endpoint target band is 5 ADC and endpoint settle/restart is 25/60 ADC.
- Minimum/recovery/maximum PWM remains 60/60/170.
- Reversal neutralization and all existing safety paths remain enabled.

---

### Task 1: Early full-PWM response and expanded range

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Test: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`

**Interfaces:**
- Consumes: target/current ADC and calibrated left/center/right endpoints.
- Produces: an overload of `computeNormalizedSteeringPwm` accepting `uint16_t fullPwmErrorPermille`; the existing overload retains 1000-permille behavior.

- [ ] **Step 1: Add failing assertions for endpoints 1020/600/180, full-output threshold 333, PWM 170 at 140 ADC error, and proportional output below that error.**
- [ ] **Step 2: Compile self-test and confirm RED from missing overload/constants and old endpoints.**
- [ ] **Step 3: Implement the overload by clamping normalized absolute error to the configured full-output threshold and scaling 60..170 over that threshold.**
- [ ] **Step 4: Change shared endpoints to 1020/600/180 and add the 333-permille configuration.**
- [ ] **Step 5: Compile self-test and require exit code 0.**

### Task 2: Endpoint-specific hold

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`
- Test: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`

**Interfaces:**
- Consumes: `isSteeringEndpointTarget(targetAdc, leftAdc, rightAdc, endpointBandAdc)`.
- Produces: endpoint target selection of 25/60 hysteresis; all other targets use 5/18.

- [ ] **Step 1: Add failing tests for left/right endpoint-band inclusion, just-outside exclusion, and 25/60 stabilizer hold across captured feedback values.**
- [ ] **Step 2: Compile self-test and confirm RED because the endpoint predicate is absent.**
- [ ] **Step 3: Implement the pure endpoint predicate and add endpoint band/settle/restart constants.**
- [ ] **Step 4: In `updateSteering`, choose endpoint or general thresholds before calling the existing stabilizer; pass full-output threshold into the PWM calculator.**
- [ ] **Step 5: Update Korean documentation with the new range, response, and endpoint hold.**
- [ ] **Step 6: Compile self-test, default RC, and ROS Uno sketches; require exit code 0 and memory within Uno limits.**
