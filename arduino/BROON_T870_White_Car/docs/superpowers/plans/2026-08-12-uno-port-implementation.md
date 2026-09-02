# BROON T870 Arduino Uno Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the existing controller from Mega-specific hardware to Arduino Uno without enabling actuator output.

**Architecture:** Preserve the public core API and replace only the board-specific pin map and RC pulse capture implementation. Uno pin-change interrupt group 1 captures A0-A2 without consuming motor PWM timers.

**Tech Stack:** Arduino Uno ATmega328P, AVR C++, Arduino AVR core 1.8.8, `arduino-cli` 1.5.0.

## Global Constraints

- Steering uses PWM D5 and DIR D4.
- Front drive uses PWM D9 and DIR D8.
- Rear drive uses PWM D11 and DIR D10.
- Encoder remains D2/D3; RC remains A0/A1/A2; potentiometer is A4.
- Actuator output remains disabled.
- ROS remains disabled for the first Uno serial bench build.
- Production RC capture must not use blocking `pulseIn()`.

---

### Task 1: Reproduce the Mega-only build failure

**Files:**
- Test: entire sketch

- [ ] Compile with `arduino-cli compile --fqbn arduino:avr:uno --warnings all .`.
- [ ] Confirm failure names `PINF` or Timer5 registers.

### Task 2: Apply the Uno motor and sensor pin map

**Files:**
- Modify: `ControllerConfig.h`

- [ ] Set front `{9, 8}`, rear `{11, 10}`, steering `{5, 4}` in PWM/DIR order.
- [ ] Keep encoder D2/D3, RC A0/A1/A2, sensor A4, LED D13.
- [ ] Update the compile-time pin assertions for Uno.

### Task 3: Replace Mega Timer5 RC capture

**Files:**
- Modify: `BroonT870Core.cpp`

- [ ] Remove Timer5 tick state, `PINF`, and `TIMER5_COMPA_vect`.
- [ ] Store rising and completed-pulse times in microseconds.
- [ ] Implement `PCINT1_vect` using `PINC & 0x07`.
- [ ] Configure `PCMSK1`, `PCIFR`, and `PCICR` in `begin()`.
- [ ] Use the existing `nowUs` argument to enforce channel timeout.

### Task 4: Verify and document the Uno build

**Files:**
- Modify: `README_KO.md`
- Modify: `CALIBRATION_RECORD_KO.md`

- [ ] Replace Mega board and pin descriptions with the confirmed Uno map.
- [ ] Compile the full default build for `arduino:avr:uno` with all warnings.
- [ ] Verify no Mega-only symbols remain and actuator output stays locked.
- [ ] Report flash and SRAM use; do not claim ROS-enabled build support until its libraries and Uno SRAM fit are independently verified.
