#pragma once

#include <stdint.h>

#include "BroonT870Core.h"

#include "BuildOptions.h"

namespace BroonT870Controller {

struct MotorPins {
  uint8_t pwm;
  uint8_t dir;
};

struct SteeringCalibration {
  int16_t leftMechanicalAdc;
  int16_t leftSafeAdc;
  int16_t centerAdc;
  int16_t rightSafeAdc;
  int16_t rightMechanicalAdc;
  bool adcIncreasesRight;
};

constexpr uint8_t kFrontEncoderAPin = 2U;
constexpr uint8_t kFrontEncoderBPin = 3U;
constexpr MotorPins kFrontDrivePins = {9U, 8U};
constexpr MotorPins kRearDrivePins = {6U, 7U};
constexpr MotorPins kSteeringPins = {5U, 4U};
constexpr uint8_t kSteeringSensorPin = A4;
constexpr uint8_t kRcSteerPin = A0;
constexpr uint8_t kRcThrottlePin = A1;
constexpr uint8_t kRcAuxPin = A2;
constexpr uint8_t kStatusLedPin = 13U;

// The current decoder counts every valid A/B transition (x4 decoding).
// Treat the supplied 75 PPR as 75 quadrature cycles per wheel revolution:
// 75 PPR * 4 edges = 300 software counts/rev. Verify this by rotating the
// Confirmed by rotating the actual front wheel once: about 300 counts.
constexpr float countsPerWheelRev = 300.0f;
// Provisional geometric circumference from the measured 270 mm diameter.
// Replace with the loaded one-revolution rollout distance for final accuracy.
constexpr float wheelCircumferenceM = 0.84823f;
// User-confirmed commanded range. Mechanical and commanded endpoints are
// intentionally identical, so there is no additional software endpoint
// margin; test the first motion with the wheels unloaded.
constexpr SteeringCalibration kSteeringCalibration = {1020, 1020, 600, 180,
                                                       180, false};

// Values transferred from the reference T870 sketch. RC wiring is unchanged:
// steering=A0, throttle=A1, mode/AUX=A2.
constexpr BroonT870::RcChannelCalibration kRcSteerCalibration = {1100, 1450,
                                                                  1800};
constexpr BroonT870::RcChannelCalibration kRcThrottleCalibration = {
    1050, 1450, 1850};
constexpr BroonT870::RcChannelCalibration kRcAuxCalibration = {1100, 1450,
                                                                1800};
constexpr uint16_t kRcThrottleDeadbandUs = 40U;
// The transmitter's throttle-cut position produces a pulse below this value.
// A missing throttle signal is also treated as an active remote stop.
constexpr uint16_t kRcStopThresholdUs = 1000U;
constexpr uint16_t kAuxRosThresholdUs = 1200U;
constexpr uint16_t kAuxRcThresholdUs = 1600U;

constexpr uint32_t kControlIntervalMs = 20UL;
constexpr uint32_t kEncoderSampleIntervalMs = 100UL;
constexpr uint32_t kStatusIntervalMs = 100UL;
constexpr uint16_t kRcTimeoutMs = 100U;
constexpr uint16_t kRosTimeoutMs = 1000U;
constexpr int16_t kRosMinimumDegrees = -25;
constexpr int16_t kRosMaximumDegrees = 25;
constexpr int16_t kRosMaximumAbsKph = 3;
constexpr float kRosSpeedKp = 20.0f;
constexpr float kRosSpeedKi = 8.0f;
constexpr float kRosSpeedDeadbandKph = 0.12f;
constexpr float kRosTargetRampKphPerSecond = 1.0f;
constexpr uint8_t kDriveFeedbackMinimumPwm = 60U;
constexpr uint16_t kDriveNoFeedbackTimeoutMs = 1500U;
constexpr float kDriveOverspeedLimitKph = 4.0f;
constexpr uint16_t kNeutralHoldMs = 500U;
constexpr uint16_t kDirectionInterlockMs = 300U;
constexpr uint16_t kSteeringNoProgressMs = 1000U;
// Zero disables the fixed continuous-runtime fault. Stall detection below
// still stops authorized PWM when the steering ADC does not make progress.
constexpr uint16_t kSteeringMaxDriveMs = 0U;
constexpr uint8_t kSteeringProgressDeltaAdc = 2U;
constexpr int16_t kSteeringElectricalMinAdc = 1;
constexpr int16_t kSteeringElectricalMaxAdc = 1022;
constexpr uint16_t kSteeringApproachBandAdc = 100U;
constexpr uint8_t kSteeringRecoveryPwm = 60U;
constexpr uint8_t kSteeringMinimumPwm = 60U;
constexpr uint8_t kSteeringMaximumPwm = 200U;
constexpr uint16_t kSteeringFullPwmErrorPermille = 250U;
constexpr uint8_t kSteeringDeadbandAdc = 8U;
constexpr uint16_t kSteeringSettleErrorAdc = 5U;
constexpr uint16_t kSteeringRestartErrorAdc = 18U;
constexpr uint16_t kSteeringEndpointTargetBandAdc = 5U;
constexpr uint16_t kSteeringEndpointSettleErrorAdc = 25U;
constexpr uint16_t kSteeringEndpointRestartErrorAdc = 60U;
constexpr uint8_t kDriveForwardMaxPwm = 80U;
constexpr uint8_t kDriveReverseMaxPwm = 80U;
constexpr uint8_t kDriveAccelerationRampStep = 10U;
constexpr uint8_t kDriveDecelerationRampStep = 20U;

constexpr bool kFrontDriveDirectionInverted = false;
constexpr bool kRearDriveDirectionInverted = false;
constexpr bool kSteeringDirectionInverted = false;

constexpr bool pinsAreDistinct() {
  return kFrontEncoderAPin != kFrontEncoderBPin &&
         kFrontEncoderAPin != kFrontDrivePins.pwm &&
         kFrontEncoderAPin != kFrontDrivePins.dir &&
         kFrontEncoderAPin != kRearDrivePins.pwm &&
         kFrontEncoderAPin != kRearDrivePins.dir &&
         kFrontEncoderAPin != kSteeringPins.pwm &&
         kFrontEncoderAPin != kSteeringPins.dir &&
         kFrontEncoderBPin != kFrontDrivePins.pwm &&
         kFrontEncoderBPin != kFrontDrivePins.dir &&
         kFrontEncoderBPin != kRearDrivePins.pwm &&
         kFrontEncoderBPin != kRearDrivePins.dir &&
         kFrontEncoderBPin != kSteeringPins.pwm &&
         kFrontEncoderBPin != kSteeringPins.dir &&
         kFrontDrivePins.pwm != kFrontDrivePins.dir &&
         kFrontDrivePins.pwm != kRearDrivePins.pwm &&
         kFrontDrivePins.pwm != kRearDrivePins.dir &&
         kFrontDrivePins.pwm != kSteeringPins.pwm &&
         kFrontDrivePins.pwm != kSteeringPins.dir &&
         kFrontDrivePins.dir != kRearDrivePins.pwm &&
         kFrontDrivePins.dir != kRearDrivePins.dir &&
         kFrontDrivePins.dir != kSteeringPins.pwm &&
         kFrontDrivePins.dir != kSteeringPins.dir &&
         kRearDrivePins.pwm != kRearDrivePins.dir &&
         kRearDrivePins.pwm != kSteeringPins.pwm &&
         kRearDrivePins.pwm != kSteeringPins.dir &&
         kRearDrivePins.dir != kSteeringPins.pwm &&
         kRearDrivePins.dir != kSteeringPins.dir &&
         kSteeringPins.pwm != kSteeringPins.dir;
}

static_assert(pinsAreDistinct(), "Motor and encoder pins must be unique");
static_assert(kFrontEncoderAPin == 2U && kFrontEncoderBPin == 3U,
              "Uno front encoder must use D2/D3");
static_assert(kFrontDrivePins.pwm == 9U && kRearDrivePins.pwm == 6U &&
                  kSteeringPins.pwm == 5U,
              "Uno motor PWM pins must match the confirmed wiring");

}  // namespace BroonT870Controller
