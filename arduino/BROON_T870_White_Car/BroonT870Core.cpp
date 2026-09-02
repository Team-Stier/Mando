#include "BroonT870Core.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

namespace {

constexpr uint16_t kRcMinimumSanePulseUs = 750U;
constexpr uint16_t kRcMaximumSanePulseUs = 2250U;
constexpr unsigned long kRcPulseInTimeoutUs = 50000UL;

volatile uint32_t gRcRiseUs[3] = {0UL, 0UL, 0UL};
volatile uint32_t gRcLastPulseUs[3] = {0UL, 0UL, 0UL};
volatile uint16_t gRcPulseUs[3] = {0U, 0U, 0U};
volatile uint8_t gRcPreviousLevels = 0U;
volatile uint8_t gRcHighSeenMask = 0U;
volatile uint8_t gRcValidMask = 0U;
volatile bool gRcCaptureEnabled = false;

int16_t clampSigned(int32_t value, int16_t minimum, int16_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return static_cast<int16_t>(value);
}

uint8_t clampMagnitude(int16_t value) {
  int32_t magnitude = value;
  if (magnitude < 0) magnitude = -magnitude;
  return static_cast<uint8_t>(magnitude > 255 ? 255 : magnitude);
}

int16_t mapLinear(int32_t value, int32_t inMinimum, int32_t inMaximum,
                  int32_t outMinimum, int32_t outMaximum) {
  if (inMaximum == inMinimum) return static_cast<int16_t>(outMinimum);
  if (value < inMinimum) value = inMinimum;
  if (value > inMaximum) value = inMaximum;
  const int32_t numerator =
      (value - inMinimum) * (outMaximum - outMinimum);
  return static_cast<int16_t>(outMinimum + numerator / (inMaximum - inMinimum));
}

int16_t rampToward(int16_t current, int16_t target, uint8_t step) {
  if (step == 0U || current == target) return target;
  if (current < target) {
    const int32_t next = static_cast<int32_t>(current) + step;
    return next > target ? target : static_cast<int16_t>(next);
  }
  const int32_t next = static_cast<int32_t>(current) - step;
  return next < target ? target : static_cast<int16_t>(next);
}

void sampleRcChannel(uint8_t index, uint8_t mask, uint8_t levels,
                     uint8_t changed, uint32_t nowUs) {
  if ((changed & mask) == 0U) return;
  if ((levels & mask) != 0U) {
    gRcRiseUs[index] = nowUs;
    gRcHighSeenMask |= mask;
    return;
  }
  if ((gRcHighSeenMask & mask) == 0U) return;
  gRcHighSeenMask &= static_cast<uint8_t>(~mask);
  const uint32_t widthUs = nowUs - gRcRiseUs[index];
  gRcLastPulseUs[index] = nowUs;
  if (widthUs >= kRcMinimumSanePulseUs &&
      widthUs <= kRcMaximumSanePulseUs) {
    gRcPulseUs[index] = static_cast<uint16_t>(widthUs);
    gRcValidMask |= mask;
  } else {
    gRcValidMask &= static_cast<uint8_t>(~mask);
  }
}

}  // namespace

ISR(PCINT1_vect) {
  if (!gRcCaptureEnabled) return;
  const uint32_t nowUs = micros();
  const uint8_t levels = PINC & 0x07U;
  const uint8_t changed = levels ^ gRcPreviousLevels;
  sampleRcChannel(0U, 0x01U, levels, changed, nowUs);
  sampleRcChannel(1U, 0x02U, levels, changed, nowUs);
  sampleRcChannel(2U, 0x04U, levels, changed, nowUs);
  gRcPreviousLevels = levels;
}

namespace BroonT870 {

bool hasElapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}

int8_t signOf(int16_t value) { return value > 0 ? 1 : (value < 0 ? -1 : 0); }

bool isRcChannelCalibrationValid(const RcChannelCalibration& calibration) {
  return calibration.minimumUs >= kRcMinimumSanePulseUs &&
         calibration.minimumUs < calibration.centerUs &&
         calibration.centerUs < calibration.maximumUs &&
         calibration.maximumUs <= kRcMaximumSanePulseUs;
}

bool isRcThrottleConfigurationValid(const RcChannelCalibration& calibration,
                                    uint16_t deadbandUs) {
  return isRcChannelCalibrationValid(calibration) && deadbandUs > 0U &&
         calibration.centerUs - calibration.minimumUs > deadbandUs &&
         calibration.maximumUs - calibration.centerUs > deadbandUs;
}

bool isAuxModeConfigurationValid(const RcChannelCalibration& calibration,
                                 uint16_t rosThresholdUs,
                                 uint16_t rcThresholdUs) {
  return isRcChannelCalibrationValid(calibration) &&
         rosThresholdUs >= calibration.minimumUs &&
         rcThresholdUs <= calibration.maximumUs &&
         rosThresholdUs < rcThresholdUs;
}

bool isRcPulseWithinCalibration(uint16_t pulseUs,
                                const RcChannelCalibration& calibration) {
  return isRcChannelCalibrationValid(calibration) &&
         pulseUs >= calibration.minimumUs && pulseUs <= calibration.maximumUs;
}

bool isSteeringCalibrationValid(int16_t leftMechanicalAdc,
                                int16_t leftSafeAdc, int16_t centerAdc,
                                int16_t rightSafeAdc,
                                int16_t rightMechanicalAdc,
                                bool adcIncreasesRight) {
  if (leftMechanicalAdc < 0 || leftMechanicalAdc > 1023 ||
      rightMechanicalAdc < 0 || rightMechanicalAdc > 1023) return false;
  if (adcIncreasesRight) {
    return leftMechanicalAdc <= leftSafeAdc && leftSafeAdc < centerAdc &&
           centerAdc < rightSafeAdc && rightSafeAdc <= rightMechanicalAdc;
  }
  return leftMechanicalAdc >= leftSafeAdc && leftSafeAdc > centerAdc &&
         centerAdc > rightSafeAdc && rightSafeAdc >= rightMechanicalAdc;
}

ControlMode updateModeFromAux(ControlMode current, uint16_t pulseUs,
                              uint16_t rosThresholdUs,
                              uint16_t rcThresholdUs) {
  if (pulseUs <= rosThresholdUs) return MODE_ROS;
  if (pulseUs >= rcThresholdUs) return MODE_RC;
  return current;
}

int16_t rcThrottleToSignedPwm(uint16_t pulseUs, uint16_t minimumUs,
                             uint16_t centerUs, uint16_t maximumUs,
                             uint16_t deadbandUs, uint8_t forwardMaximumPwm,
                             uint8_t reverseMaximumPwm) {
  if (minimumUs >= centerUs || centerUs >= maximumUs) return 0;
  const uint16_t lowNeutral = centerUs - deadbandUs;
  const uint16_t highNeutral = centerUs + deadbandUs;
  if (pulseUs < lowNeutral) {
    return mapLinear(pulseUs, minimumUs, lowNeutral,
                     -static_cast<int16_t>(reverseMaximumPwm), 0);
  }
  if (pulseUs > highNeutral) {
    return mapLinear(pulseUs, highNeutral, maximumUs, 0,
                     forwardMaximumPwm);
  }
  return 0;
}

int16_t rcSteerToTargetAdc(uint16_t pulseUs, uint16_t minimumUs,
                           uint16_t maximumUs, int16_t leftAdc,
                           int16_t rightAdc) {
  return mapLinear(pulseUs, minimumUs, maximumUs, leftAdc, rightAdc);
}

int16_t rcSteerToTargetAdc(uint16_t pulseUs, uint16_t minimumUs,
                           uint16_t centerUs, uint16_t maximumUs,
                           int16_t leftAdc, int16_t centerAdc,
                           int16_t rightAdc) {
  if (minimumUs >= centerUs || centerUs >= maximumUs) return centerAdc;
  if (pulseUs <= centerUs)
    return mapLinear(pulseUs, minimumUs, centerUs, leftAdc, centerAdc);
  return mapLinear(pulseUs, centerUs, maximumUs, centerAdc, rightAdc);
}

int16_t kphToSignedPwm(int16_t kph, int16_t maximumAbsKph,
                       uint8_t forwardMaximumPwm,
                       uint8_t reverseMaximumPwm) {
  if (maximumAbsKph <= 0) return 0;
  kph = clampSigned(kph, -maximumAbsKph, maximumAbsKph);
  if (kph >= 0)
    return mapLinear(kph, 0, maximumAbsKph, 0, forwardMaximumPwm);
  return mapLinear(kph, -maximumAbsKph, 0,
                   -static_cast<int16_t>(reverseMaximumPwm), 0);
}

int16_t degreesToSteerAdc(int16_t degrees, int16_t minimumDegrees,
                          int16_t maximumDegrees, int16_t leftAdc,
                          int16_t centerAdc, int16_t rightAdc) {
  if (minimumDegrees >= 0 || maximumDegrees <= 0) return centerAdc;
  if (degrees < 0)
    return mapLinear(degrees, minimumDegrees, 0, leftAdc, centerAdc);
  return mapLinear(degrees, 0, maximumDegrees, centerAdc, rightAdc);
}

VehicleCommand selectActiveCommand(const VehicleCommand& rcCommand,
                                   const VehicleCommand& rosCommand,
                                   ControlMode selectedMode, bool auxValid) {
  VehicleCommand selected = selectedMode == MODE_ROS ? rosCommand : rcCommand;
  selected.mode = selectedMode;
  if (!auxValid || !selected.valid) {
    selected.drivePwm = 0;
    selected.targetSpeedKph = 0.0f;
    selected.valid = false;
  }
  return selected;
}

SafetyResult updateSafetyState(SafetyState& state, const SafetyInputs& inputs,
                               uint32_t nowMs) {
  FaultCode fault = state.latchedFault;
  if (fault == FAULT_NONE && inputs.steeringSensorFault)
    fault = FAULT_STEERING_SENSOR;
  if (fault == FAULT_NONE && inputs.steeringStallFault)
    fault = FAULT_STEERING_STALL;
  if (fault == FAULT_NONE && inputs.steeringRuntimeFault)
    fault = FAULT_STEERING_RUNTIME;
  if (fault == FAULT_NONE && inputs.driveNoFeedbackFault)
    fault = FAULT_DRIVE_NO_FEEDBACK;
  if (fault == FAULT_NONE && inputs.driveOverspeedFault)
    fault = FAULT_DRIVE_OVERSPEED;
  if (fault == FAULT_NONE && inputs.actuatorOutputsEnabled &&
      (!inputs.steeringCalibrationConfirmed || !inputs.configurationValid))
    fault = FAULT_CONFIGURATION;
  if (fault != FAULT_NONE) {
    state.latchedFault = fault;
    state.state = STATE_FAULT_LATCHED;
    state.neutralTiming = false;
    return {state.state, fault, false, true};
  }

  const bool stop = inputs.remoteStopActive || !inputs.commandValid ||
                    inputs.brakeRequested;
  if (!inputs.actuatorOutputsEnabled) {
    state.state = STATE_BOOT_LOCKED;
    state.neutralTiming = false;
    return {state.state, FAULT_NONE, false, true};
  }
  if (stop) {
    state.state = STATE_DISARMED;
    state.neutralTiming = false;
    return {state.state, FAULT_NONE, false, true};
  }
  if (state.state == STATE_ARMED)
    return {state.state, FAULT_NONE, true, false};
  state.state = STATE_DISARMED;
  if (!inputs.throttleNeutral) {
    state.neutralTiming = false;
    return {state.state, FAULT_NONE, false, false};
  }
  if (!state.neutralTiming) {
    state.neutralTiming = true;
    state.neutralSinceMs = nowMs;
  }
  if (hasElapsed(nowMs, state.neutralSinceMs, inputs.neutralHoldMs)) {
    state.state = STATE_ARMED;
    state.neutralTiming = false;
    return {state.state, FAULT_NONE, true, false};
  }
  return {state.state, FAULT_NONE, false, false};
}

int16_t steeringPositionPermille(int16_t adc, int16_t leftAdc,
                                 int16_t centerAdc, int16_t rightAdc) {
  const int16_t leftSpan = leftAdc - centerAdc;
  const int16_t rightSpan = rightAdc - centerAdc;
  if (leftSpan == 0 || rightSpan == 0 ||
      (static_cast<int32_t>(leftSpan) * rightSpan) >= 0)
    return 0;

  const int16_t offset = adc - centerAdc;
  const bool onLeftSide =
      (leftSpan > 0 && offset >= 0) || (leftSpan < 0 && offset <= 0);
  int32_t position = 0;
  if (onLeftSide) {
    position = static_cast<int32_t>(offset) * 1000L / leftSpan;
    if (position < 0) position = 0;
    if (position > 1000) position = 1000;
  } else {
    position = -static_cast<int32_t>(offset) * 1000L / rightSpan;
    if (position > 0) position = 0;
    if (position < -1000) position = -1000;
  }
  return static_cast<int16_t>(position);
}

int16_t computeNormalizedSteeringPwm(
    int16_t targetAdc, int16_t currentAdc, int16_t leftAdc,
    int16_t centerAdc, int16_t rightAdc, uint8_t deadbandAdc,
    uint8_t minimumPwm, uint8_t maximumPwm) {
  return computeNormalizedSteeringPwm(
      targetAdc, currentAdc, leftAdc, centerAdc, rightAdc, deadbandAdc,
      minimumPwm, maximumPwm, 1000U);
}

int16_t computeNormalizedSteeringPwm(
    int16_t targetAdc, int16_t currentAdc, int16_t leftAdc,
    int16_t centerAdc, int16_t rightAdc, uint8_t deadbandAdc,
    uint8_t minimumPwm, uint8_t maximumPwm,
    uint16_t fullPwmErrorPermille) {
  if (maximumPwm == 0U || minimumPwm > maximumPwm ||
      fullPwmErrorPermille == 0U || fullPwmErrorPermille > 1000U ||
      leftAdc == centerAdc || rightAdc == centerAdc ||
      (static_cast<int32_t>(leftAdc - centerAdc) *
       (rightAdc - centerAdc)) >= 0)
    return 0;

  int16_t adcError = targetAdc - currentAdc;
  if (adcError < 0) adcError = -adcError;
  if (adcError <= deadbandAdc) return 0;

  const int16_t targetPosition =
      steeringPositionPermille(targetAdc, leftAdc, centerAdc, rightAdc);
  const int16_t currentPosition =
      steeringPositionPermille(currentAdc, leftAdc, centerAdc, rightAdc);
  int16_t positionError = targetPosition - currentPosition;
  int16_t absoluteError = positionError < 0 ? -positionError : positionError;
  if (absoluteError > static_cast<int16_t>(fullPwmErrorPermille))
    absoluteError = static_cast<int16_t>(fullPwmErrorPermille);
  const int32_t magnitude =
      minimumPwm + static_cast<int32_t>(maximumPwm - minimumPwm) *
                       absoluteError / fullPwmErrorPermille;
  return positionError > 0 ? -static_cast<int16_t>(magnitude)
                           : static_cast<int16_t>(magnitude);
}

void resetSteeringStabilizer(SteeringStabilizerState& state) {
  state = initialSteeringStabilizerState();
}

int16_t stabilizeSteeringPwm(SteeringStabilizerState& state,
                             int16_t requestedPwm, int16_t targetAdc,
                             int16_t currentAdc, uint16_t settleErrorAdc,
                             uint16_t restartErrorAdc,
                             uint8_t restartConfirmSamples) {
  if (settleErrorAdc == 0U || restartErrorAdc <= settleErrorAdc ||
      restartConfirmSamples == 0U) {
    resetSteeringStabilizer(state);
    return 0;
  }

  int32_t absoluteError = static_cast<int32_t>(targetAdc) - currentAdc;
  if (absoluteError < 0) absoluteError = -absoluteError;
  if (absoluteError <= settleErrorAdc) {
    state.settled = true;
    state.lastDirection = 0;
    state.restartSamples = 0U;
    return 0;
  }
  if (state.settled) {
    if (absoluteError < restartErrorAdc) {
      state.restartSamples = 0U;
      return 0;
    }
    if (state.restartSamples < restartConfirmSamples)
      ++state.restartSamples;
    if (state.restartSamples < restartConfirmSamples) return 0;
    state.restartSamples = 0U;
  }
  state.settled = false;

  const int8_t requestedDirection = signOf(requestedPwm);
  if (requestedDirection == 0) {
    state.lastDirection = 0;
    state.restartSamples = 0U;
    return 0;
  }
  if (state.lastDirection != 0 &&
      requestedDirection != state.lastDirection) {
    state.lastDirection = 0;
    // A small inertial target crossing is a settled condition, not a reason
    // to immediately reverse the motor and start hunting around the target.
    if (absoluteError < restartErrorAdc) {
      state.settled = true;
      state.restartSamples = 0U;
    }
    return 0;
  }
  state.lastDirection = requestedDirection;
  return requestedPwm;
}

bool isSteeringEndpointTarget(int16_t targetAdc, int16_t leftEndpointAdc,
                              int16_t rightEndpointAdc,
                              uint16_t endpointBandAdc) {
  if (leftEndpointAdc < 0 || leftEndpointAdc > 1023 ||
      rightEndpointAdc < 0 || rightEndpointAdc > 1023 ||
      leftEndpointAdc == rightEndpointAdc || endpointBandAdc > 1023U)
    return false;
  int32_t leftDistance = static_cast<int32_t>(targetAdc) - leftEndpointAdc;
  if (leftDistance < 0) leftDistance = -leftDistance;
  int32_t rightDistance = static_cast<int32_t>(targetAdc) - rightEndpointAdc;
  if (rightDistance < 0) rightDistance = -rightDistance;
  return leftDistance <= endpointBandAdc ||
         rightDistance <= endpointBandAdc;
}

SteeringGuardResult guardSteeringPwm(int16_t requestedPwm,
                                     int16_t currentAdc,
                                     int16_t leftSafeAdc,
                                     int16_t rightSafeAdc,
                                     uint16_t approachBandAdc,
                                     uint8_t maximumPwm,
                                     uint8_t minimumPwm,
                                     uint8_t recoveryPwm) {
  if (currentAdc < 0 || currentAdc > 1023)
    return {0, true, false};
  if (leftSafeAdc < 0 || leftSafeAdc > 1023 || rightSafeAdc < 0 ||
      rightSafeAdc > 1023 || leftSafeAdc == rightSafeAdc || maximumPwm == 0U ||
      minimumPwm > maximumPwm)
    return {0, false, true};

  requestedPwm = clampSigned(requestedPwm, -maximumPwm, maximumPwm);
  const bool increasesRight = leftSafeAdc < rightSafeAdc;
  const bool beyondLeft = increasesRight ? currentAdc <= leftSafeAdc
                                         : currentAdc >= leftSafeAdc;
  const bool beyondRight = increasesRight ? currentAdc >= rightSafeAdc
                                          : currentAdc <= rightSafeAdc;
  if (beyondLeft) {
    if (requestedPwm <= 0) return {0, false, false};
    return {static_cast<int16_t>(requestedPwm > recoveryPwm ? recoveryPwm
                                                            : requestedPwm),
            false, false};
  }
  if (beyondRight) {
    if (requestedPwm >= 0) return {0, false, false};
    const int16_t limit = -static_cast<int16_t>(recoveryPwm);
    return {requestedPwm < limit ? limit : requestedPwm, false, false};
  }
  if (approachBandAdc == 0U || requestedPwm == 0) return {requestedPwm, false, false};

  int16_t distance = 0;
  if (requestedPwm > 0)
    distance = increasesRight ? rightSafeAdc - currentAdc
                              : currentAdc - rightSafeAdc;
  else
    distance = increasesRight ? currentAdc - leftSafeAdc
                              : leftSafeAdc - currentAdc;
  if (distance >= static_cast<int16_t>(approachBandAdc))
    return {requestedPwm, false, false};
  const int32_t scaled = static_cast<int32_t>(requestedPwm) * distance /
                         static_cast<int32_t>(approachBandAdc);
  int16_t allowedPwm = static_cast<int16_t>(scaled);
  if (requestedPwm > 0 && allowedPwm < minimumPwm)
    allowedPwm = minimumPwm;
  if (requestedPwm < 0 && allowedPwm > -static_cast<int16_t>(minimumPwm))
    allowedPwm = -static_cast<int16_t>(minimumPwm);
  return {allowedPwm, false, false};
}

void resetSteeringWatchdog(SteeringWatchdogState& state,
                           int16_t currentAdc, uint32_t nowMs) {
  state.active = false;
  state.referenceAdc = currentAdc;
  state.startedAtMs = nowMs;
  state.progressAtMs = nowMs;
  state.lastDirection = 0;
}

SteeringWatchdogResult updateSteeringWatchdog(
    SteeringWatchdogState& state, int16_t currentAdc, int16_t authorizedPwm,
    uint8_t progressDeltaAdc, uint16_t noProgressMs, uint16_t maxDriveMs,
    uint32_t nowMs) {
  const int8_t direction = signOf(authorizedPwm);
  if (direction == 0) {
    resetSteeringWatchdog(state, currentAdc, nowMs);
    return {false, false};
  }
  if (!state.active || state.lastDirection != direction) {
    state.active = true;
    state.referenceAdc = currentAdc;
    state.startedAtMs = nowMs;
    state.progressAtMs = nowMs;
    state.lastDirection = direction;
    return {false, false};
  }
  int16_t movement = currentAdc - state.referenceAdc;
  if (movement < 0) movement = -movement;
  if (movement >= progressDeltaAdc) {
    state.referenceAdc = currentAdc;
    state.progressAtMs = nowMs;
  }
  return {noProgressMs > 0U &&
              hasElapsed(nowMs, state.progressAtMs, noProgressMs),
          steeringRuntimeLimitExpired(nowMs, state.startedAtMs, maxDriveMs)};
}

MotorOutputPlan planMotorOutput(MotorOutputState& state, int16_t logicalPwm,
                                bool invertDirection, bool authorized) {
  if (!authorized || logicalPwm == 0)
    return {0U, state.directionHigh, false};
  bool direction = logicalPwm > 0;
  if (invertDirection) direction = !direction;
  const bool change = direction != state.directionHigh;
  state.directionHigh = direction;
  return {clampMagnitude(logicalPwm), direction, change};
}

void resetDrivePair(DrivePairState& state, uint32_t nowMs) {
  state.frontPwm = 0;
  state.rearPwm = 0;
  state.pendingDirection = 0;
  state.interlockStartedAtMs = nowMs;
  state.interlockActive = false;
}

DrivePairResult updateDrivePair(DrivePairState& state, int16_t targetPwm,
                                uint8_t forwardMaximumPwm,
                                uint8_t reverseMaximumPwm,
                                uint8_t accelerationRampStep,
                                uint8_t decelerationRampStep,
                                uint16_t directionInterlockMs,
                                uint32_t nowMs, bool immediateStop) {
  if (immediateStop) {
    resetDrivePair(state, nowMs);
    return {0, 0};
  }
  targetPwm = clampSigned(targetPwm, -reverseMaximumPwm, forwardMaximumPwm);
  const int8_t targetDirection = signOf(targetPwm);
  if (targetDirection == 0) {
    resetDrivePair(state, nowMs);
    return {0, 0};
  }
  const int8_t currentDirection =
      signOf(state.frontPwm != 0 ? state.frontPwm : state.rearPwm);
  if (!state.interlockActive && currentDirection != 0 &&
      targetDirection != 0 && currentDirection != targetDirection) {
    state.frontPwm = 0;
    state.rearPwm = 0;
    state.pendingDirection = targetDirection;
    state.interlockStartedAtMs = nowMs;
    state.interlockActive = true;
    return {0, 0};
  }
  if (state.interlockActive) {
    if (targetDirection == 0 || targetDirection != state.pendingDirection) {
      state.interlockActive = false;
      state.pendingDirection = 0;
      return {0, 0};
    }
    if (!hasElapsed(nowMs, state.interlockStartedAtMs,
                    directionInterlockMs))
      return {0, 0};
    state.interlockActive = false;
    state.pendingDirection = 0;
  }
  int16_t frontMagnitude = state.frontPwm;
  if (frontMagnitude < 0) frontMagnitude = -frontMagnitude;
  int16_t rearMagnitude = state.rearPwm;
  if (rearMagnitude < 0) rearMagnitude = -rearMagnitude;
  int16_t targetMagnitude = targetPwm;
  if (targetMagnitude < 0) targetMagnitude = -targetMagnitude;
  const uint8_t frontStep =
      currentDirection == targetDirection && targetMagnitude < frontMagnitude
          ? decelerationRampStep
          : accelerationRampStep;
  const uint8_t rearStep =
      currentDirection == targetDirection && targetMagnitude < rearMagnitude
          ? decelerationRampStep
          : accelerationRampStep;
  state.frontPwm = rampToward(state.frontPwm, targetPwm, frontStep);
  state.rearPwm = rampToward(state.rearPwm, targetPwm, rearStep);
  return {state.frontPwm, state.rearPwm};
}

int8_t quadratureStep(uint8_t previousState, uint8_t currentState) {
  static const int8_t transitions[16] = {0,  -1, 1,  0, 1, 0,  0, -1,
                                          -1, 0,  0,  1, 0, 1, -1, 0};
  return transitions[((previousState & 0x03U) << 2U) |
                     (currentState & 0x03U)];
}

float rpmFromCountDelta(long deltaCount, float countsPerRevolution,
                        uint32_t sampleTimeMs) {
  if (countsPerRevolution <= 0.0f || sampleTimeMs == 0UL) return 0.0f;
  return static_cast<float>(deltaCount) * 60000.0f /
         (countsPerRevolution * static_cast<float>(sampleTimeMs));
}

float speedMpsFromRpm(float rpm, float wheelCircumferenceM) {
  if (wheelCircumferenceM <= 0.0f) return 0.0f;
  return rpm * wheelCircumferenceM / 60.0f;
}

void resetSpeedPi(SpeedPiState& state) {
  state.integralPwm = 0.0f;
  state.rampedTargetKph = 0.0f;
}

SpeedPiResult updateSpeedPi(SpeedPiState& state, float requestedKph,
                            float measuredKph, float kp, float ki,
                            float deadbandKph,
                            float targetRampKphPerSecond,
                            uint8_t maximumPwm, uint32_t sampleTimeMs) {
  if (requestedKph <= 0.0f || kp < 0.0f || ki < 0.0f ||
      deadbandKph < 0.0f || targetRampKphPerSecond <= 0.0f ||
      maximumPwm == 0U || sampleTimeMs == 0UL) {
    resetSpeedPi(state);
    return {0, 0.0f};
  }

  const float seconds = static_cast<float>(sampleTimeMs) / 1000.0f;
  const float maximumTargetChange = targetRampKphPerSecond * seconds;
  if (state.rampedTargetKph < requestedKph) {
    state.rampedTargetKph += maximumTargetChange;
    if (state.rampedTargetKph > requestedKph)
      state.rampedTargetKph = requestedKph;
  } else if (state.rampedTargetKph > requestedKph) {
    state.rampedTargetKph -= maximumTargetChange;
    if (state.rampedTargetKph < requestedKph)
      state.rampedTargetKph = requestedKph;
  }

  if (measuredKph < 0.0f) measuredKph = -measuredKph;
  float error = state.rampedTargetKph - measuredKph;
  if (error >= -deadbandKph && error <= deadbandKph) error = 0.0f;

  const float candidateIntegral = state.integralPwm + ki * error * seconds;
  const float candidateOutput = kp * error + candidateIntegral;
  const bool pushesAboveMaximum =
      candidateOutput > static_cast<float>(maximumPwm) && error > 0.0f;
  const bool pushesBelowMinimum = candidateOutput < 0.0f && error < 0.0f;
  if (!pushesAboveMaximum && !pushesBelowMinimum)
    state.integralPwm = candidateIntegral;

  float output = kp * error + state.integralPwm;
  if (output < 0.0f) output = 0.0f;
  if (output > static_cast<float>(maximumPwm))
    output = static_cast<float>(maximumPwm);
  return {static_cast<int16_t>(output + 0.5f), state.rampedTargetKph};
}

void resetDriveFeedbackWatchdog(DriveFeedbackWatchdogState& state) {
  state.noFeedbackSinceMs = 0UL;
  state.timing = false;
}

DriveFeedbackWatchdogResult updateDriveFeedbackWatchdog(
    DriveFeedbackWatchdogState& state, float targetKph,
    float measuredAbsoluteKph, long encoderDeltaCount, int16_t appliedPwm,
    uint8_t minimumMonitoredPwm, uint16_t noFeedbackTimeoutMs,
    float overspeedLimitKph, uint32_t nowMs) {
  if (measuredAbsoluteKph < 0.0f)
    measuredAbsoluteKph = -measuredAbsoluteKph;
  const bool overspeedFault =
      overspeedLimitKph > 0.0f && measuredAbsoluteKph > overspeedLimitKph;

  int32_t pwmMagnitude = appliedPwm;
  if (pwmMagnitude < 0) pwmMagnitude = -pwmMagnitude;
  if (targetKph <= 0.0f ||
      pwmMagnitude < static_cast<int32_t>(minimumMonitoredPwm) ||
      noFeedbackTimeoutMs == 0U) {
    resetDriveFeedbackWatchdog(state);
    return {false, overspeedFault};
  }
  if (encoderDeltaCount != 0L) {
    resetDriveFeedbackWatchdog(state);
    return {false, overspeedFault};
  }
  if (!state.timing) {
    state.noFeedbackSinceMs = nowMs;
    state.timing = true;
    return {false, overspeedFault};
  }
  return {hasElapsed(nowMs, state.noFeedbackSinceMs, noFeedbackTimeoutMs),
          overspeedFault};
}

FeedbackStatus nextFeedbackStatus(bool remoteStopActive, bool stopRequested,
                                  uint8_t previousAlive) {
  return {static_cast<uint8_t>(remoteStopActive ? 1U : 0U),
          static_cast<uint8_t>(stopRequested ? 1U : 0U),
          static_cast<uint8_t>(previousAlive + 1U)};
}

BroonT870RcInput::BroonT870RcInput()
    : initialized_(false), steerPin_(A0), throttlePin_(A1), auxPin_(A2) {}

void BroonT870RcInput::begin(uint8_t steerPin, uint8_t throttlePin,
                            uint8_t auxPin) {
  initialized_ = steerPin == A0 && throttlePin == A1 && auxPin == A2;
  if (!initialized_) return;
  steerPin_ = steerPin;
  throttlePin_ = throttlePin;
  auxPin_ = auxPin;
  pinMode(steerPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(auxPin, INPUT);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    gRcCaptureEnabled = false;
    gRcValidMask = 0U;
    gRcHighSeenMask = 0U;
    PCMSK1 &= static_cast<uint8_t>(~0x07U);
  }
}

RcSnapshot BroonT870RcInput::snapshot(uint32_t nowUs,
                                      uint16_t timeoutMs) const {
  if (!initialized_)
    return {0U, 0U, 0U, false, false, false};
  (void)nowUs;
  (void)timeoutMs;
  const unsigned long steerPulseUs =
      pulseIn(steerPin_, HIGH, kRcPulseInTimeoutUs);
  const unsigned long throttlePulseUs =
      pulseIn(throttlePin_, HIGH, kRcPulseInTimeoutUs);
  const unsigned long auxPulseUs =
      pulseIn(auxPin_, HIGH, kRcPulseInTimeoutUs);
  const bool steerValid = steerPulseUs >= kRcMinimumSanePulseUs &&
                          steerPulseUs <= kRcMaximumSanePulseUs;
  const bool throttleValid = throttlePulseUs >= kRcMinimumSanePulseUs &&
                             throttlePulseUs <= kRcMaximumSanePulseUs;
  const bool auxValid = auxPulseUs >= kRcMinimumSanePulseUs &&
                        auxPulseUs <= kRcMaximumSanePulseUs;
  return {static_cast<uint16_t>(steerPulseUs),
          static_cast<uint16_t>(throttlePulseUs),
          static_cast<uint16_t>(auxPulseUs), steerValid, throttleValid,
          auxValid};
}

}  // namespace BroonT870
