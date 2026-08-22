# Steering Hunting Suppression and Compact RC Log Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop continuous steering direction chatter near the target while preserving full distant response, and reduce the human RC status line to relevant fields.

**Architecture:** Add a small stateful steering stabilizer to the hardware-independent core. It gates the existing normalized PWM with settle/restart hysteresis and a one-update neutral output on direction reversal; the main sketch continues to apply endpoint guards and watchdogs afterward. Compact only the non-ROS serial formatting path.

**Tech Stack:** Arduino AVR C++11, Arduino Uno, arduino-cli self-test sketch

## Global Constraints

- Do not change RC or steering calibration.
- Keep steering PWM limits at 60..140.
- Settle at absolute ADC error <= 15 and restart at absolute ADC error >= 30.
- Insert exactly one zero-output control update before an opposite nonzero direction.
- Preserve safety stops, endpoint guards, steering watchdogs, drive behavior, and ROS transport.
- Human RC log retains state/fault/mode/stop, RC steering/throttle, drive request/front/rear, steering target/current/PWM, measured kph, and RC read time.

---

### Task 1: Stateful steering stabilization

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `ControllerConfig.h`
- Test: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`

**Interfaces:**
- Consumes: existing signed normalized steering PWM, target ADC, current ADC.
- Produces: `SteeringStabilizerState` and `stabilizeSteeringPwm(state, requestedPwm, targetAdc, currentAdc, settleErrorAdc, restartErrorAdc)` returning the authorized signed PWM for this control update.

- [ ] **Step 1: Add failing self-tests**

Cover entry into settle at 15 ADC, remaining settled at 29 ADC, restarting at 30 ADC, passing full PWM far away, producing one zero update on sign reversal, and passing the opposite PWM on the following update.

- [ ] **Step 2: Compile to verify RED**

Run:
`arduino-cli compile --fqbn arduino:avr:uno --library . tests/BroonT870ControlSelfTest`

Expected: compilation fails because the stabilizer interface does not exist.

- [ ] **Step 3: Implement the minimal core state machine**

Add state fields for settled status, last applied direction, and pending reversed PWM. Validate thresholds by returning zero when settle is zero or restart is not greater than settle. Never delay a zero request.

- [ ] **Step 4: Compile to verify GREEN**

Run the same self-test compile and expect exit code 0.

### Task 2: Integrate stabilizer and compact the RC log

**Files:**
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`
- Test: `tests/BroonT870ControlSelfTest/BroonT870ControlSelfTest.ino`

**Interfaces:**
- Consumes: Task 1 stabilizer and `kSteeringSettleErrorAdc=15`, `kSteeringRestartErrorAdc=30`.
- Produces: stabilized PWM passed into the existing endpoint guard, plus one compact human-readable status line.

- [ ] **Step 1: Route normalized PWM through the stabilizer**

Store one global stabilizer state. Reset it when all outputs are forced off. Pass the stabilized result to `guardSteeringPwm`; keep all later authorization, watchdog, and motor writes unchanged.

- [ ] **Step 2: Replace the human serial field list**

Emit `state`, `fault`, `mode`, `stop`, `rc_s`, `rc_t`, `drive=t/f/r`, `steer=t/a/p`, `kph`, and `read_us`. Remove ROS-only speed target/PI output, encoder total, steering min/max, and raw AUX from this line.

- [ ] **Step 3: Verify all Uno builds**

Compile the self-test, default RC sketch, and ROS sketch with the installed ros_lib. Each command must exit 0 and remain within Uno flash/SRAM limits.

- [ ] **Step 4: Document bench interpretation**

Document that a persistent valid receiver failsafe pulse can still display an ADC target near 730 after transmitter loss, and that transmitter presence cannot be inferred from an unchanged valid PWM alone.
