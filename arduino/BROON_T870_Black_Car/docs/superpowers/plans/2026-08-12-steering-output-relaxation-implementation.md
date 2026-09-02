# Steering Output Relaxation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Increase steering breakaway force and reduce nuisance stall faults without removing mechanical end protection.

**Architecture:** Keep the existing P controller and steering guard. Change only centralized tuning constants, expose guarded/authorized steering PWM in the human log, and pin the new constants with compile-time self-test assertions.

**Tech Stack:** Arduino Uno AVR C++, Arduino CLI, serial self-test sketch.

## Global Constraints

- Steering PWM range is 40 through 80.
- Progress is 2 ADC within 1000 ms.
- Maximum continuous steering drive is 2500 ms.
- Recovery PWM is 40.
- Keep Kp 0.25, deadband 8 ADC, approach band 100 ADC, and safe endpoints 850/170.
- Do not change RC calibration, motor pin map, or output lock values.

---

### Task 1: Pin the relaxed steering configuration

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `ControllerConfig.h`

**Interfaces:**
- Consumes: `BroonT870Controller` steering tuning constants.
- Produces: compile-time guarantees for the selected PWM and watchdog values.

- [x] **Step 1: Add static assertions for PWM 40/80, progress 2, no-progress 1000 ms, runtime 2500 ms, and recovery PWM 40.**
- [x] **Step 2: Compile the self-test and verify the assertions fail against the old constants.**
- [x] **Step 3: Change only the matching constants in `ControllerConfig.h`.**
- [x] **Step 4: Compile the self-test and verify success.**

### Task 2: Expose steering PWM diagnostics

**Files:**
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`

**Interfaces:**
- Produces: `steer_pwm=<guarded>/<authorized>` in human-readable status output.

- [x] **Step 1: Store the latest guarded and authorized steering PWM values.**
- [x] **Step 2: Reset both diagnostic values on immediate stop or controller fault.**
- [x] **Step 3: Print both values next to steering target/current ADC.**
- [x] **Step 4: Document the new limits and fault-3 interpretation.**

### Task 3: Fresh verification

**Files:**
- Verify: all modified controller and test files.

**Interfaces:**
- Consumes: completed Tasks 1 and 2.
- Produces: Uno compile evidence for self-test, default, and ROS configurations.

- [x] **Step 1: Compile the self-test with warnings enabled.**
- [x] **Step 2: Compile the default Uno controller with warnings enabled.**
- [x] **Step 3: Compile the ROS Uno controller with warnings enabled.**
- [x] **Step 4: Confirm RC calibration, pin map, and output-lock values did not change.**
