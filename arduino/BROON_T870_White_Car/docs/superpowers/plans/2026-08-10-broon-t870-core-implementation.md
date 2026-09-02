# Broon T870 Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the missing local `BroonT870Core` dependency so the Mega controller compiles and retains fail-safe RC/ROS, steering, drive, and encoder behavior.

**Architecture:** Add a focused local header/source pair. Pure mapping and safety functions remain independent of Arduino I/O; `BroonT870RcInput` alone owns Mega-specific A0/A1/A2 capture. Because Mega A0-A2 lack external/pin-change interrupts, Timer5 samples port F every 50 us without blocking the 20 ms control loop; Timer5 PWM pins D44-D46 remain unused.

**Tech Stack:** Arduino Mega 2560, AVR C++, Arduino core, Timer5 CTC interrupt, `arduino-cli` 1.5.0.

## Global Constraints

- Keep actuator output locked by default.
- RC inputs are A0 steer, A1 throttle, A2 AUX.
- A1 missing or below 1000 us activates RC remote stop in both RC and ROS modes.
- AUX below 1200 us selects ROS; above 1600 us selects RC.
- Drive pins are front PWM D10/DIR D11 and rear PWM D6/DIR D7.
- Steering pins are PWM D8/DIR D9 with potentiometer A5.
- Do not use blocking `pulseIn()` in the production control loop.
- Do not mix human-readable serial output with rosserial.

---

### Task 1: Define the Core API and make the missing-header build advance

**Files:**
- Create: `BroonT870Core.h`
- Test: production sketch compile

**Interfaces:**
- Produces all enums, structs, class declarations, and function signatures currently referenced as `BroonT870::*`.

- [ ] Create declarations for control modes, controller states, fault codes, RC calibration/snapshot, vehicle command, safety state/result, motor output, drive pair, steering guard/watchdog, encoder measurement, and feedback status.
- [ ] Declare every function extracted from the current `.ino` and `RosBridge.h` references.
- [ ] Run `arduino-cli compile --fqbn arduino:avr:mega .`.
- [ ] Verify failure advances from “header missing” to undefined implementations, proving the sketch resolves the local header.

### Task 2: Implement pure calibration and command mapping

**Files:**
- Create: `BroonT870Core.cpp`
- Modify: `BroonT870Core.h`
- Test: production sketch compile

**Interfaces:**
- Produces `hasElapsed`, `signOf`, calibration validators, RC pulse range checks, AUX hysteresis, RC throttle/steer mapping, ROS KPH/degrees mapping, and active-command selection.

- [ ] Implement overflow-safe elapsed-time comparison using unsigned subtraction.
- [ ] Validate strict `minimum < center < maximum` ordering and deadband bounds.
- [ ] Map throttle independently on each side of center and clamp outputs to configured forward/reverse maxima.
- [ ] Map steering using left/center/right ADC points without assuming ADC increases right.
- [ ] Require valid AUX for either selected command; never reuse a stale command from the other mode.
- [ ] Compile and confirm these symbols resolve.

### Task 3: Implement fail-safe safety state machine

**Files:**
- Modify: `BroonT870Core.cpp`
- Test: production sketch compile plus output-locked bench observation

**Interfaces:**
- Consumes `SafetyState`, `SafetyInputs`, and current milliseconds.
- Produces `SafetyResult updateSafetyState(...)`.

- [ ] Keep boot/disarmed/fault states output-disabled.
- [ ] Make configuration failure, RC remote stop, invalid command, brake, or steering fault request immediate zero output.
- [ ] Latch steering faults until reboot.
- [ ] Require a continuously valid neutral command for `kNeutralHoldMs` before arming.
- [ ] Reset neutral timing whenever any prerequisite is lost.
- [ ] Compile and confirm the existing main loop initializes all fields in the declared order.

### Task 4: Implement steering control and protection

**Files:**
- Modify: `BroonT870Core.cpp`
- Test: compile; later output-locked A5 calibration and lifted-wheel test

**Interfaces:**
- Produces steering calibration validation, P output, endpoint guard, watchdog reset/update.

- [ ] Validate five ordered steering calibration points for either ADC direction.
- [ ] Compute signed P control with deadband, minimum effective PWM, and maximum PWM clamp.
- [ ] Block motion farther outside left/right safe limits.
- [ ] Within the approach band, reduce output as the safe endpoint approaches.
- [ ] Outside the safe interval, permit only inward recovery capped by recovery PWM.
- [ ] Run stall and maximum-runtime timers only while nonzero steering PWM is actually authorized.
- [ ] Reset watchdog timing whenever output is zero or direction changes.

### Task 5: Implement drive output planning and interlock

**Files:**
- Modify: `BroonT870Core.cpp`
- Test: compile; later lifted-wheel front/rear direction test

**Interfaces:**
- Produces motor output planning, drive-pair reset, ramping, and reversal interlock.

- [ ] Force PWM zero when unauthorized.
- [ ] Change DIR only while PWM is zero.
- [ ] Ramp front and rear PWM independently by configured per-tick steps.
- [ ] On sign reversal, hold both outputs at zero for the full interlock interval before applying opposite-direction PWM.
- [ ] Bypass ramps and immediately return zero when the safety input requests a stop.

### Task 6: Implement encoder and feedback helpers

**Files:**
- Modify: `BroonT870Core.cpp`
- Test: compile; output-locked hand rotation

**Interfaces:**
- Produces quadrature transition decoding, RPM/speed conversions, alive counter, and feedback flags.

- [ ] Use a 16-entry quadrature transition table returning -1, 0, or +1.
- [ ] Return zero RPM for nonpositive counts/rev or zero sample interval.
- [ ] Convert RPM to m/s using wheel circumference and 60 seconds/minute.
- [ ] Increment `uint8_t` alive with natural wraparound.

### Task 7: Implement nonblocking Mega RC capture

**Files:**
- Modify: `BroonT870Core.h`
- Modify: `BroonT870Core.cpp`
- Modify: `BROON_T870_Uno_Controller.ino`
- Test: compile; serial RC raw-pulse bench test

**Interfaces:**
- Produces `BroonT870RcInput::begin(steerPin, throttlePin, auxPin)` and `snapshot(nowUs, timeoutMs)`.

- [ ] Change the main sketch to pass A0/A1/A2 explicitly to `begin()`.
- [ ] Reject any pin combination other than Mega A0/A1/A2 so miswiring fails safe.
- [ ] Configure Timer5 CTC at 20 kHz and sample PINF bits 0-2 in `TIMER5_COMPA_vect`.
- [ ] Record rising tick, validated pulse width 750-2250 us, and last completed pulse tick independently per channel.
- [ ] Copy ISR-owned multi-byte values inside `ATOMIC_BLOCK` before constructing a snapshot.
- [ ] Mark a channel invalid when its last completed pulse is older than `timeoutMs`.
- [ ] Compile and confirm Timer5 does not overlap motor PWM timers/pins.

### Task 8: Integration verification and diagnostics

**Files:**
- Modify: `BROON_T870_Uno_Controller.ino`
- Modify: `README_KO.md`

**Interfaces:**
- Adds output-locked logs for RC pulse widths, mode, RC stop, A5 ADC, and encoder count.

- [ ] Compile default human-serial build for `arduino:avr:mega` with zero warnings treated as build failures where supported.
- [ ] Verify the binary leaves actuator output disabled by checking `BROON_ENABLE_ACTUATOR_OUTPUTS=0`.
- [ ] Upload and observe A5 steering ADC while moving only by hand.
- [ ] Observe A0/A1/A2 pulses, AUX hysteresis, A1 throttle-cut, and receiver-power-loss invalidation.
- [ ] Rotate the front wheel ten turns and record D2/D3 count and sign.
- [ ] Keep ROS disabled until RC, steering calibration, and lifted-wheel direction tests pass.

## Self-review

- The plan covers every currently referenced `BroonT870` symbol.
- No physical A4 E-stop dependency is reintroduced.
- RC sampling is nonblocking despite Mega A0-A2 lacking external interrupts.
- Steering protection is implemented before actuator output can be authorized.
- Full ROS-enabled compilation remains a later gate because `ros_lib` and `erp42_msgs` are not currently installed.
