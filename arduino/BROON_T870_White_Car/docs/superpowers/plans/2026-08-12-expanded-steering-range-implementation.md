# Expanded Steering Range Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expand the commanded steering endpoints to ADC 980 and 280 while retaining measured mechanical endpoints and existing protections.

**Architecture:** Change the single shared steering calibration consumed by RC mapping, ROS mapping, normalized PWM, and endpoint guarding. Update behavior tests and current operator records to the same values.

**Tech Stack:** Arduino AVR C++, Arduino Uno, `arduino-cli`

## Global Constraints

- Calibration is mechanical left 999, commanded left 980, center 640, commanded right 280, mechanical right 21.
- RC steering maps 1100/1450/1800 microseconds to ADC 980/640/280.
- Steering PWM remains 60 through 140.
- Fault 3 remains 2 ADC within 1000 ms; fault 4 remains disabled.
- Pin mapping, RC pulse calibration, drive control, and output locks do not change.

---

### Task 1: Change shared steering calibration

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `ControllerConfig.h`
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`

**Interfaces:**
- Consumes: `BroonT870Controller::kSteeringCalibration`.
- Produces: shared endpoints 980/640/280 for RC, ROS, normalized PWM, and endpoint guard paths.

- [x] **Step 1: Change behavior tests to require RC endpoints/midpoints 980/810/640/460/280, normalized endpoints 980/640/280, and endpoint guarding at 980/280.**
- [x] **Step 2: Compile the self-test and confirm failure against the old 850/170 production calibration.**
- [x] **Step 3: Change only the shared commanded endpoints in `ControllerConfig.h` to 980 and 280.**
- [x] **Step 4: Update README and calibration records to the new current range and mechanical margins.**
- [x] **Step 5: Compile the self-test, RC sketch, and ROS sketch for Arduino Uno and audit all unchanged safety and pin settings.**
