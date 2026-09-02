# Steering Centering and Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce opposite-approach center-position spread to at most 10 ADC counts ideally and increase far-error steering speed.

**Architecture:** Retune only the existing steering stabilizer thresholds and symmetric normalized PWM ceiling. Preserve the current state machine, safety guards, minimum/recovery PWM, calibration, and RC/ROS command paths.

**Tech Stack:** Arduino AVR C++11, Arduino Uno, arduino-cli

## Global Constraints

- `kSteeringSettleErrorAdc=5`.
- `kSteeringRestartErrorAdc=18`.
- `kSteeringMaximumPwm=170`.
- Keep minimum and recovery PWM at 60.
- Do not change calibration, reversal neutralization, endpoint guards, or faults.

---

### Task 1: Retune steering configuration

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `ControllerConfig.h`
- Modify: `README_KO.md`

**Interfaces:**
- Consumes: existing `stabilizeSteeringPwm` and `computeNormalizedSteeringPwm`.
- Produces: new shared compile-time thresholds and maximum PWM used by RC and ROS steering.

- [ ] **Step 1: Change assertions to require settle 5, restart 18, and maximum PWM 170; change full-error PWM expectations from ±140 to ±170.**
- [ ] **Step 2: Compile the self-test and confirm it fails against the old 15/30/140 configuration.**
- [ ] **Step 3: Change only the three configuration constants in `ControllerConfig.h`.**
- [ ] **Step 4: Update `README_KO.md` to describe the 5/18 hysteresis and 60..170 PWM range.**
- [ ] **Step 5: Compile self-test, default RC, and ROS Uno builds and require exit code 0 for all three.**
