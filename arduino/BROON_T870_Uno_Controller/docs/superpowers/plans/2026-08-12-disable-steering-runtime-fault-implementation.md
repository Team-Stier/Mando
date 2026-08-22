# Disable Steering Runtime Fault Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Disable fault 4 when the steering motor remains active while retaining the stall and endpoint protections.

**Architecture:** Add an explicit, testable zero-disables contract for the steering runtime timer. Configure this vehicle with a zero limit while leaving the existing fault API available for future configurations.

**Tech Stack:** Arduino AVR C++, Arduino Uno, `arduino-cli`

## Global Constraints

- Keep fault 3 at 2 ADC of required progress within 1000 ms.
- Keep safe steering endpoints at ADC 850 and 170.
- Do not change steering PWM, RC calibration, pin mapping, or drive control.

---

### Task 1: Make the runtime limit optional

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Test: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`

**Interfaces:**
- Consumes: steering watchdog start time, current time, and maximum runtime.
- Produces: `steeringRuntimeLimitExpired(nowMs, startedAtMs, maxDriveMs)` where zero disables expiration.

- [x] **Step 1: Add a compile-time behavior test for the zero-disabled runtime limit and require the vehicle configuration to use zero.**
- [x] **Step 2: Compile the self-test and verify it fails because the behavior is not implemented.**
- [x] **Step 3: Implement the zero-disabled predicate, use it in the watchdog, and set `kSteeringMaxDriveMs` to zero.**
- [x] **Step 4: Update the operator documentation to state that fault 4 is disabled while fault 3 remains active.**
- [x] **Step 5: Compile the self-test, standard RC sketch, and ROS sketch for Arduino Uno.**
