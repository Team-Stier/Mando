# T870 Steering and ROS Speed Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add conservative potentiometer-based steering P control and front-encoder-based ROS PI speed control without enabling physical actuator outputs.

**Status (2026-08-12):** Implemented and compile-verified for the Uno self-test, default controller, and ROS controller. Runtime assertions and physical motor tests remain intentionally unrun; both actuator locks remain `0`.

**Architecture:** Keep hardware capture and safety orchestration in the existing controller, while adding deterministic mapping, PI, and watchdog functions to `BroonT870Core`. RC keeps direct limited PWM; ROS carries a target KPH through `VehicleCommand`, computes PWM from measured front-wheel speed every encoder sample, then reuses the existing paired-drive ramp and direction interlock.

**Tech Stack:** Arduino Uno AVR C++11, rosserial `erp42_msgs`, Arduino CLI, serial self-test sketch.

## Global Constraints

- Keep `BROON_ENABLE_ACTUATOR_OUTPUTS=0` and `BROON_STEERING_CALIBRATION_CONFIRMED=0`.
- Use 300 encoder counts/rev and provisional wheel circumference 0.84823 m.
- Steering calibration is mechanical left 999, safe left 850, center 640, safe right 170, mechanical right 21, with ADC decreasing to the right.
- Steering P values are Kp 0.25, deadband 8 ADC, minimum PWM 25, maximum PWM 45, approach band 100 ADC, recovery PWM 25.
- ROS target speed is forward-only and capped at 3 km/h; drive PWM is capped at 80/255.
- RC mode remains direct throttle-to-PWM control.
- Physical runtime tests require a lifted wheel and a reachable 24 V disconnect and are not performed automatically.

---

### Task 1: Piecewise Steering Mapping and Calibration

**Files:**
- Create: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`

**Interfaces:**
- Produces: `int16_t rcSteerToTargetAdc(uint16_t pulseUs, uint16_t minimumUs, uint16_t centerUs, uint16_t maximumUs, int16_t leftAdc, int16_t centerAdc, int16_t rightAdc)`.
- Preserves: existing overload taking minimum, maximum, left, and right values for compatibility until all callers migrate.

- [ ] **Step 1: Write the failing steering mapping self-test**

Add a serial self-test sketch that includes `../../BroonT870Core.cpp`, calls the new seven-argument mapping API, and asserts `1100 -> 850`, `1450 -> 640`, `1800 -> 170`, plus midpoint values on both sides.

- [ ] **Step 2: Compile the self-test and verify RED**

Run:

```powershell
arduino-cli compile --fqbn arduino:avr:uno tests/BroonT870ControlSelfTest
```

Expected: compile failure because the center-aware overload does not exist.

- [ ] **Step 3: Implement the center-aware mapping**

Clamp the pulse to the calibrated minimum/maximum. Map `minimumUs..centerUs` to `leftAdc..centerAdc` and `centerUs..maximumUs` to `centerAdc..rightAdc`; return `centerAdc` for invalid calibration ordering.

- [ ] **Step 4: Enter measured steering and wheel configuration**

Set `wheelCircumferenceM=0.84823f`, `kSteeringCalibration={999,850,640,170,21,false}`, approach/recovery/min/max/Kp/deadband to the global constraints, and forward/reverse drive caps to 80. Do not change either build lock.

- [ ] **Step 5: Compile the self-test and verify GREEN**

Expected: successful AVR compile. Runtime assertions will be run only after explicit authorization to upload the self-test.

### Task 2: Pure PI Speed Controller

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`

**Interfaces:**
- Produces: `SpeedPiState { float integralPwm; float rampedTargetKph; }`.
- Produces: `SpeedPiResult { int16_t pwm; float rampedTargetKph; }`.
- Produces: `resetSpeedPi(SpeedPiState&)` and `updateSpeedPi(SpeedPiState&, float requestedKph, float measuredKph, float kp, float ki, float deadbandKph, float targetRampKphPerSecond, uint8_t maximumPwm, uint32_t sampleTimeMs)`.

- [ ] **Step 1: Add failing PI compile tests**

Add tests for reset, target ramp limiting, nonnegative/capped output, integral accumulation below saturation, anti-windup at maximum output, and immediate zero/reset for a zero target.

- [ ] **Step 2: Compile and verify RED**

Expected: missing `SpeedPiState` and PI functions.

- [ ] **Step 3: Implement minimal PI logic**

Ramp target by `targetRampKphPerSecond * dt`, calculate `error = rampedTarget - abs(measured)`, suppress the proportional error inside the speed deadband, integrate using seconds, clamp output to `0..maximumPwm`, and accept integral updates only when not pushing farther into saturation. Reset for zero/invalid targets or invalid configuration.

- [ ] **Step 4: Add conservative PI constants**

Use ROS cap 3 km/h, `Kp=20 PWM/(km/h)`, `Ki=8 PWM/(km/h*s)`, speed deadband 0.12 km/h, and target ramp 1.0 km/h/s. Keep these constants centralized in `ControllerConfig.h` for bench tuning.

- [ ] **Step 5: Compile and verify GREEN**

Expected: successful AVR compile with all serial self-test cases linked.

### Task 3: Drive Feedback Safety Watchdog

**Files:**
- Modify: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`

**Interfaces:**
- Produces: fault codes `FAULT_DRIVE_NO_FEEDBACK` and `FAULT_DRIVE_OVERSPEED`.
- Produces: `DriveFeedbackWatchdogState { uint32_t noFeedbackSinceMs; bool timing; }`.
- Produces: `DriveFeedbackWatchdogResult { bool noFeedbackFault; bool overspeedFault; }`.
- Produces: `resetDriveFeedbackWatchdog` and `updateDriveFeedbackWatchdog` consuming target KPH, measured absolute KPH, encoder delta, applied PI PWM, PWM threshold, no-feedback timeout, absolute overspeed limit, and current milliseconds.

- [ ] **Step 1: Add failing watchdog tests**

Cover reset while stopped, timer start only above PWM threshold, no fault before 1500 ms, fault at/after 1500 ms with zero counts, timer reset when a count arrives, and overspeed fault above 4.0 km/h.

- [ ] **Step 2: Compile and verify RED**

Expected: missing watchdog types and functions.

- [ ] **Step 3: Implement watchdog state machine**

Make target zero or PWM below 60 reset no-feedback timing. Any nonzero encoder delta resets timing. Continuous zero delta for 1500 ms at/above PWM 60 faults. Measured absolute speed above 4.0 km/h faults independently.

- [ ] **Step 4: Compile and verify GREEN**

Expected: successful AVR compile.

### Task 4: Integrate ROS Target Speed and Controller State

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `RosBridge.h`
- Modify: `BROON_T870_Uno_Controller.ino`

**Interfaces:**
- Changes: `VehicleCommand` gains `float targetSpeedKph`; RC sets it to zero, ROS sets it from capped `DriveCmd.KPH`.
- Consumes: PI and watchdog APIs from Tasks 2 and 3.

- [ ] **Step 1: Add compile assertions through the self-test and main build**

Update test initializers and add a test that `selectActiveCommand` preserves the ROS target KPH. Compile before changing production initializers and verify failures identify the new field/API mismatch.

- [ ] **Step 2: Make ROS bridge carry target speed instead of open-loop PWM**

Remove ROS KPH-to-PWM conversion from `RosBridge::vehicleCommand`. Cap KPH at 3, assign `targetSpeedKph`, keep `drivePwm=0`, and retain unsigned forward-only behavior.

- [ ] **Step 3: Integrate encoder-first PI updates**

Sample the encoder before drive calculation, set a measurement-updated flag, run PI only on new 100 ms samples, store the resulting ROS PWM, and feed that PWM to the existing paired drive logic only in ROS mode. Reset PI immediately on stop, brake, timeout, RC mode, or invalid command.

- [ ] **Step 4: Integrate safety faults**

Feed the watchdog results into `updateSafetyState`, latch the two new drive faults, and ensure every fault path forces all three motor outputs to zero.

- [ ] **Step 5: Integrate center-aware RC steering**

Change the RC call to pass minimum/center/maximum pulse values and left/center/right ADC values. ROS already uses piecewise degree mapping around center 640.

- [ ] **Step 6: Extend diagnostics**

Human serial output reports target KPH, measured KPH, ROS PI PWM, steering target ADC, and current ADC. ROS feedback continues reporting measured m/s and signed encoder delta without human-readable text on the ROS serial port.

- [ ] **Step 7: Compile default and ROS builds**

Run the Uno default build and ROS build with `BROON_ENABLE_ROS=1`, `BROON_ENABLE_HUMAN_SERIAL=0`. Both must exit 0 and remain within Uno flash/SRAM limits.

### Task 5: Documentation and Final Verification

**Files:**
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`
- Modify: `docs/superpowers/plans/2026-08-12-steering-and-ros-speed-control-implementation.md`

**Interfaces:**
- Documents all operator-visible limits and the remaining physical calibration step.

- [ ] **Step 1: Record measured calibration**

Document 300 count/rev, 270 mm diameter, provisional 0.84823 m circumference, mechanical/safe/center steering ADC values, and conservative PWM limits.

- [ ] **Step 2: Document controller behavior and bench procedure**

Explain steering P versus speed PI, ROS forward-only 3 km/h limit, RC open-loop behavior, watchdog fault meanings, and how to replace calculated circumference with loaded rollout distance.

- [ ] **Step 3: Run fresh verification**

Compile the self-test, default controller, and ROS controller. Search build options to prove both actuator locks remain zero. Review the final changed-file list; this workspace is not a Git repository, so no commits are attempted.
