#pragma once

#include <stdint.h>

#include <BroonT870Core.h>

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
constexpr MotorPins kFrontDrivePins = {5U, 4U};
constexpr MotorPins kRearDrivePins = {6U, 7U};
constexpr MotorPins kSteeringPins = {9U, 8U};
constexpr uint8_t kSteeringSensorPin = A0;
constexpr uint8_t kRcSteerPin = A1;
constexpr uint8_t kRcThrottlePin = A2;
constexpr uint8_t kRcAuxPin = A3;
constexpr uint8_t kEstopPin = A4;
constexpr uint8_t kStatusLedPin = A5;
constexpr uint8_t kReservedSpiFirstPin = 10U;
constexpr uint8_t kReservedSpiLastPin = 13U;

// Both entries remain explicitly unconfirmed until the vehicle's raw A4
// levels are measured independently for released and active E-stop states.
constexpr BroonT870::EstopConfiguration kEstopConfiguration = {
    BroonT870::ESTOP_POLARITY_UNCONFIRMED,
    BroonT870::ESTOP_POLARITY_UNCONFIRMED,
    BROON_ESTOP_POLARITY_CONFIRMED != 0};

constexpr float countsPerWheelRev = 0.0f;
constexpr float wheelCircumferenceM = 0.0f;
constexpr SteeringCalibration kSteeringCalibration = {0, 0, 0, 0, 0, false};

// Provisional transfer defaults preserve the previous 1100/1450/1800
// behavior. Replace each channel independently from the signed vehicle record.
constexpr BroonT870::RcChannelCalibration kRcSteerCalibration = {1100, 1450,
                                                                  1800};
constexpr BroonT870::RcChannelCalibration kRcThrottleCalibration = {
    1100, 1450, 1800};
constexpr BroonT870::RcChannelCalibration kRcAuxCalibration = {1100, 1450,
                                                                1800};
constexpr uint16_t kRcThrottleDeadbandUs = 40U;
constexpr uint16_t kAuxRosThresholdUs = 1200U;
constexpr uint16_t kAuxRcThresholdUs = 1600U;

constexpr uint32_t kControlIntervalMs = 20UL;
constexpr uint32_t kEncoderSampleIntervalMs = 100UL;
constexpr uint32_t kStatusIntervalMs = 100UL;
constexpr uint16_t kRcTimeoutMs = 100U;
constexpr uint16_t kRosTimeoutMs = 1000U;
constexpr int16_t kRosMinimumDegrees = -25;
constexpr int16_t kRosMaximumDegrees = 25;
constexpr int16_t kRosMaximumAbsKph = 25;
constexpr uint16_t kNeutralHoldMs = 500U;
constexpr uint16_t kDirectionInterlockMs = 300U;
constexpr uint16_t kSteeringNoProgressMs = 500U;
constexpr uint16_t kSteeringMaxDriveMs = 1500U;
constexpr uint8_t kSteeringProgressDeltaAdc = 4U;
constexpr int16_t kSteeringElectricalMinAdc = 1;
constexpr int16_t kSteeringElectricalMaxAdc = 1022;
constexpr uint16_t kSteeringApproachBandAdc = 0U;
constexpr uint8_t kSteeringRecoveryPwm = 0U;
constexpr uint8_t kSteeringMinimumPwm = 0U;
constexpr uint8_t kSteeringMaximumPwm = 0U;
constexpr float kSteeringKp = 0.0f;
constexpr uint8_t kSteeringDeadbandAdc = 0U;
constexpr uint8_t kDriveForwardMaxPwm = 0U;
constexpr uint8_t kDriveReverseMaxPwm = 0U;
constexpr uint8_t kFrontDriveRampStep = 1U;
constexpr uint8_t kRearDriveRampStep = 1U;

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
static_assert(kFrontDrivePins.pwm == 5U && kRearDrivePins.pwm == 6U &&
                  kSteeringPins.pwm == 9U,
              "Motor PWM pins must remain hardware PWM pins");

}  // namespace BroonT870Controller
