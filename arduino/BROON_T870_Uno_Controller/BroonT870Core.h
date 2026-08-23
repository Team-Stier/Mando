#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace BroonT870 {

enum ControlMode : uint8_t { MODE_RC = 0, MODE_ROS = 1 };
enum ControllerState : uint8_t {
  STATE_BOOT_LOCKED = 0,
  STATE_DISARMED = 1,
  STATE_ARMED = 2,
  STATE_FAULT_LATCHED = 3
};
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_CONFIGURATION = 1,
  FAULT_STEERING_SENSOR = 2,
  FAULT_STEERING_STALL = 3,
  FAULT_STEERING_RUNTIME = 4,
  FAULT_DRIVE_NO_FEEDBACK = 5,
  FAULT_DRIVE_OVERSPEED = 6
};

struct RcChannelCalibration {
  uint16_t minimumUs;
  uint16_t centerUs;
  uint16_t maximumUs;
};

struct RcSnapshot {
  uint16_t steerPulseUs;
  uint16_t throttlePulseUs;
  uint16_t auxPulseUs;
  bool steerValid;
  bool throttleValid;
  bool auxValid;
};

struct VehicleCommand {
  int16_t steerTargetAdc;
  int16_t drivePwm;
  float targetSpeedKph;
  ControlMode mode;
  uint32_t receivedAtMs;
  bool valid;
  bool brakeRequested;
};

struct SafetyState {
  ControllerState state;
  FaultCode latchedFault;
  uint32_t neutralSinceMs;
  bool neutralTiming;
};

struct SafetyInputs {
  bool actuatorOutputsEnabled;
  bool steeringCalibrationConfirmed;
  bool configurationValid;
  bool remoteStopActive;
  bool commandValid;
  bool brakeRequested;
  bool throttleNeutral;
  bool steeringSensorFault;
  bool steeringStallFault;
  bool steeringRuntimeFault;
  bool driveNoFeedbackFault;
  bool driveOverspeedFault;
  ControlMode selectedMode;
  uint16_t neutralHoldMs;
};

struct SafetyResult {
  ControllerState state;
  FaultCode activeFault;
  bool outputsAllowed;
  bool immediateStop;
};

struct MotorOutputState { bool directionHigh; };

struct MotorOutputPlan {
  uint8_t pwm;
  bool directionHigh;
  bool directionWriteRequired;
};

struct DrivePairState {
  int16_t frontPwm;
  int16_t rearPwm;
  int8_t pendingDirection;
  uint32_t interlockStartedAtMs;
  bool interlockActive;
};

struct DrivePairResult {
  int16_t frontPwm;
  int16_t rearPwm;
};

struct SteeringGuardResult {
  int16_t allowedPwm;
  bool sensorFault;
  bool configurationFault;
};

struct SteeringWatchdogState {
  bool active;
  int16_t referenceAdc;
  uint32_t startedAtMs;
  uint32_t progressAtMs;
  int8_t lastDirection;
};

struct SteeringStabilizerState {
  bool settled;
  int8_t lastDirection;
};

constexpr SteeringStabilizerState initialSteeringStabilizerState() {
  return {false, 0};
}

struct SteeringWatchdogResult {
  bool stallFault;
  bool runtimeFault;
};

struct EncoderMeasurement {
  long deltaCount;
  long totalCount;
  float rpm;
  float speedMps;
  bool calibrated;
};

struct SpeedPiState {
  float integralPwm;
  float rampedTargetKph;
};

struct SpeedPiResult {
  int16_t pwm;
  float rampedTargetKph;
};

struct DriveFeedbackWatchdogState {
  uint32_t noFeedbackSinceMs;
  bool timing;
};

struct DriveFeedbackWatchdogResult {
  bool noFeedbackFault;
  bool overspeedFault;
};

struct FeedbackStatus {
  uint8_t estop;
  uint8_t brake;
  uint8_t alive;
};

bool hasElapsed(uint32_t now, uint32_t since, uint32_t interval);
int8_t signOf(int16_t value);
bool isRcChannelCalibrationValid(const RcChannelCalibration& calibration);
bool isRcThrottleConfigurationValid(const RcChannelCalibration& calibration,
                                    uint16_t deadbandUs);
bool isAuxModeConfigurationValid(const RcChannelCalibration& calibration,
                                 uint16_t rosThresholdUs,
                                 uint16_t rcThresholdUs);
bool isRcPulseWithinCalibration(uint16_t pulseUs,
                                const RcChannelCalibration& calibration);
bool isSteeringCalibrationValid(int16_t leftMechanicalAdc,
                                int16_t leftSafeAdc, int16_t centerAdc,
                                int16_t rightSafeAdc,
                                int16_t rightMechanicalAdc,
                                bool adcIncreasesRight);
ControlMode updateModeFromAux(ControlMode current, uint16_t pulseUs,
                              uint16_t rosThresholdUs,
                              uint16_t rcThresholdUs);
int16_t rcThrottleToSignedPwm(uint16_t pulseUs, uint16_t minimumUs,
                             uint16_t centerUs, uint16_t maximumUs,
                             uint16_t deadbandUs, uint8_t forwardMaximumPwm,
                             uint8_t reverseMaximumPwm);
int16_t rcSteerToTargetAdc(uint16_t pulseUs, uint16_t minimumUs,
                           uint16_t maximumUs, int16_t leftAdc,
                           int16_t rightAdc);
int16_t rcSteerToTargetAdc(uint16_t pulseUs, uint16_t minimumUs,
                           uint16_t centerUs, uint16_t maximumUs,
                           int16_t leftAdc, int16_t centerAdc,
                           int16_t rightAdc);
int16_t kphToSignedPwm(int16_t kph, int16_t maximumAbsKph,
                       uint8_t forwardMaximumPwm,
                       uint8_t reverseMaximumPwm);
int16_t degreesToSteerAdc(int16_t degrees, int16_t minimumDegrees,
                          int16_t maximumDegrees, int16_t leftAdc,
                          int16_t centerAdc, int16_t rightAdc);
VehicleCommand selectActiveCommand(const VehicleCommand& rcCommand,
                                   const VehicleCommand& rosCommand,
                                   ControlMode selectedMode, bool auxValid);
SafetyResult updateSafetyState(SafetyState& state, const SafetyInputs& inputs,
                               uint32_t nowMs);
int16_t steeringPositionPermille(int16_t adc, int16_t leftAdc,
                                 int16_t centerAdc, int16_t rightAdc);
int16_t computeNormalizedSteeringPwm(
    int16_t targetAdc, int16_t currentAdc, int16_t leftAdc,
    int16_t centerAdc, int16_t rightAdc, uint8_t deadbandAdc,
    uint8_t minimumPwm, uint8_t maximumPwm);
int16_t computeNormalizedSteeringPwm(
    int16_t targetAdc, int16_t currentAdc, int16_t leftAdc,
    int16_t centerAdc, int16_t rightAdc, uint8_t deadbandAdc,
    uint8_t minimumPwm, uint8_t maximumPwm,
    uint16_t fullPwmErrorPermille);
void resetSteeringStabilizer(SteeringStabilizerState& state);
int16_t stabilizeSteeringPwm(SteeringStabilizerState& state,
                             int16_t requestedPwm, int16_t targetAdc,
                             int16_t currentAdc, uint16_t settleErrorAdc,
                             uint16_t restartErrorAdc);
bool isSteeringEndpointTarget(int16_t targetAdc, int16_t leftEndpointAdc,
                              int16_t rightEndpointAdc,
                              uint16_t endpointBandAdc);
SteeringGuardResult guardSteeringPwm(int16_t requestedPwm,
                                     int16_t currentAdc,
                                     int16_t leftSafeAdc,
                                     int16_t rightSafeAdc,
                                     uint16_t approachBandAdc,
                                     uint8_t maximumPwm,
                                     uint8_t minimumPwm,
                                     uint8_t recoveryPwm);
constexpr bool steeringRuntimeLimitExpired(uint32_t nowMs,
                                           uint32_t startedAtMs,
                                           uint16_t maxDriveMs) {
  return maxDriveMs != 0U &&
         static_cast<uint32_t>(nowMs - startedAtMs) >= maxDriveMs;
}
void resetSteeringWatchdog(SteeringWatchdogState& state,
                           int16_t currentAdc, uint32_t nowMs);
SteeringWatchdogResult updateSteeringWatchdog(
    SteeringWatchdogState& state, int16_t currentAdc, int16_t authorizedPwm,
    uint8_t progressDeltaAdc, uint16_t noProgressMs, uint16_t maxDriveMs,
    uint32_t nowMs);
MotorOutputPlan planMotorOutput(MotorOutputState& state, int16_t logicalPwm,
                                bool invertDirection, bool authorized);
void resetDrivePair(DrivePairState& state, uint32_t nowMs);
DrivePairResult updateDrivePair(DrivePairState& state, int16_t targetPwm,
                                uint8_t forwardMaximumPwm,
                                uint8_t reverseMaximumPwm,
                                uint8_t accelerationRampStep,
                                uint8_t decelerationRampStep,
                                uint16_t directionInterlockMs,
                                uint32_t nowMs, bool immediateStop);
int8_t quadratureStep(uint8_t previousState, uint8_t currentState);
float rpmFromCountDelta(long deltaCount, float countsPerRevolution,
                        uint32_t sampleTimeMs);
float speedMpsFromRpm(float rpm, float wheelCircumferenceM);
void resetSpeedPi(SpeedPiState& state);
SpeedPiResult updateSpeedPi(SpeedPiState& state, float requestedKph,
                            float measuredKph, float kp, float ki,
                            float deadbandKph,
                            float targetRampKphPerSecond,
                            uint8_t minimumPwm, uint8_t maximumPwm,
                            uint32_t sampleTimeMs);
void resetDriveFeedbackWatchdog(DriveFeedbackWatchdogState& state);
DriveFeedbackWatchdogResult updateDriveFeedbackWatchdog(
    DriveFeedbackWatchdogState& state, float targetKph,
    float measuredAbsoluteKph, long encoderDeltaCount, int16_t appliedPwm,
    uint8_t minimumMonitoredPwm, uint16_t noFeedbackTimeoutMs,
    float overspeedLimitKph, uint32_t nowMs);
FeedbackStatus nextFeedbackStatus(bool remoteStopActive, bool stopRequested,
                                  uint8_t previousAlive);

class BroonT870RcInput {
 public:
  BroonT870RcInput();
  void begin(uint8_t steerPin, uint8_t throttlePin, uint8_t auxPin);
  RcSnapshot snapshot(uint32_t nowUs, uint16_t timeoutMs) const;

 private:
  bool initialized_;
  uint8_t steerPin_;
  uint8_t throttlePin_;
  uint8_t auxPin_;
};

}  // namespace BroonT870
