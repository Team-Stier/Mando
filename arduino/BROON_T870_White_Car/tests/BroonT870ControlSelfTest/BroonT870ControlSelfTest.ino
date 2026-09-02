#include <Arduino.h>

#include <BroonT870Core.h>
#include <ControllerConfig.h>

static_assert(BroonT870Controller::kSteeringMinimumPwm == 70U,
              "steering minimum PWM must provide breakaway force");
static_assert(BroonT870Controller::kSteeringMaximumPwm == 220U,
              "steering maximum PWM must be 220");
static_assert(BroonT870Controller::kSteeringRecoveryPwm == 70U,
              "steering recovery PWM must provide breakaway force");
static_assert(BroonT870Controller::kSteeringProgressDeltaAdc == 2U,
              "two ADC counts must be accepted as steering progress");
static_assert(BroonT870Controller::kSteeringNoProgressMs == 0U,
              "steering stall latch must be disabled for bring-up");
static_assert(BroonT870Controller::kSteeringMaxDriveMs == 0U,
              "zero must disable the steering runtime limit");
static_assert(
    BroonT870Controller::kSteeringCalibration.leftMechanicalAdc == 1023 &&
        BroonT870Controller::kSteeringCalibration.leftSafeAdc == 1000 &&
        BroonT870Controller::kSteeringCalibration.centerAdc == 512 &&
        BroonT870Controller::kSteeringCalibration.rightSafeAdc == 20 &&
        BroonT870Controller::kSteeringCalibration.rightMechanicalAdc == 0,
    "steering calibration must use the current vehicle range and margin");
static_assert(BroonT870Controller::kSteeringFullPwmErrorPermille == 250U,
              "steering must reach full PWM at one quarter normalized error");
static_assert(BroonT870Controller::kSteeringApproachBandAdc == 60U,
              "endpoint slowdown must not cover too much steering travel");
static_assert(BroonT870Controller::kDriveAccelerationRampStep == 10U,
              "drive acceleration must respond in ten-PWM steps");
static_assert(BroonT870Controller::kDriveDecelerationRampStep == 20U,
              "drive deceleration must respond faster than acceleration");
static_assert(BroonT870Controller::kSteeringSettleErrorAdc == 12U,
              "steering must settle across small potentiometer jitter");
static_assert(BroonT870Controller::kSteeringRestartErrorAdc == 25U,
              "steering must not restart on small feedback jitter");
static_assert(BroonT870Controller::kSteeringRestartConfirmSamples == 3U,
              "steering restart must reject brief feedback excursions");
static_assert(BroonT870Controller::kSteeringEndpointTargetBandAdc == 5U,
              "endpoint target detection band must be five ADC counts");
static_assert(BroonT870Controller::kSteeringEndpointSettleErrorAdc == 25U,
              "endpoint hold must settle within twenty-five ADC counts");
static_assert(BroonT870Controller::kSteeringEndpointRestartErrorAdc == 60U,
              "endpoint hold must ignore the captured feedback bounce");
static_assert(!BroonT870::initialSteeringStabilizerState().settled,
              "steering reset must begin unsettled");
static_assert(!BroonT870::steeringRuntimeLimitExpired(5000UL, 0UL, 0U),
              "disabled steering runtime limit must never expire");
static_assert(!BroonT870::steeringRuntimeLimitExpired(2499UL, 0UL, 2500U),
              "enabled steering runtime limit must wait for its deadline");
static_assert(BroonT870::steeringRuntimeLimitExpired(2500UL, 0UL, 2500U),
              "enabled steering runtime limit must expire at its deadline");

namespace {

uint16_t passCount = 0U;
uint16_t failCount = 0U;

void expectEqual(const __FlashStringHelper* name, long actual, long expected) {
  if (actual == expected) {
    ++passCount;
    Serial.print(F("PASS "));
  } else {
    ++failCount;
    Serial.print(F("FAIL "));
  }
  Serial.print(name);
  Serial.print(F(" actual="));
  Serial.print(actual);
  Serial.print(F(" expected="));
  Serial.println(expected);
}

void expectNear(const __FlashStringHelper* name, float actual, float expected,
                float tolerance) {
  const float difference = actual > expected ? actual - expected
                                             : expected - actual;
  if (difference <= tolerance) {
    ++passCount;
    Serial.print(F("PASS "));
  } else {
    ++failCount;
    Serial.print(F("FAIL "));
  }
  Serial.print(name);
  Serial.print(F(" actual="));
  Serial.print(actual, 4);
  Serial.print(F(" expected="));
  Serial.println(expected, 4);
}

void testCenterAwareRcSteeringMapping() {
  expectEqual(F("zero-margin steering calibration accepted"),
              BroonT870::isSteeringCalibrationValid(
                  1020, 1020, 600, 180, 180, false),
              1);
  expectEqual(F("steer left endpoint"),
              BroonT870::rcSteerToTargetAdc(1100U, 1100U, 1450U, 1800U,
                                             1020, 600, 180),
              1020);
  expectEqual(F("steer center"),
              BroonT870::rcSteerToTargetAdc(1450U, 1100U, 1450U, 1800U,
                                             1020, 600, 180),
              600);
  expectEqual(F("steer right endpoint"),
              BroonT870::rcSteerToTargetAdc(1800U, 1100U, 1450U, 1800U,
                                             1020, 600, 180),
              180);
  expectEqual(F("steer left midpoint"),
              BroonT870::rcSteerToTargetAdc(1275U, 1100U, 1450U, 1800U,
                                             1020, 600, 180),
              810);
  expectEqual(F("steer right midpoint"),
              BroonT870::rcSteerToTargetAdc(1625U, 1100U, 1450U, 1800U,
                                             1020, 600, 180),
              390);
}

void testResponsiveDriveRampAndNeutralStop() {
  BroonT870::DrivePairState state;
  BroonT870::resetDrivePair(state, 0UL);

  BroonT870::DrivePairResult result = BroonT870::updateDrivePair(
      state, 80, 80U, 80U, 10U, 20U, 300U, 0UL, false);
  expectEqual(F("drive acceleration step front"), result.frontPwm, 10);
  expectEqual(F("drive acceleration step rear"), result.rearPwm, 10);

  state.frontPwm = 80;
  state.rearPwm = 80;
  result = BroonT870::updateDrivePair(state, 40, 80U, 80U, 10U, 20U,
                                      300U, 20UL, false);
  expectEqual(F("drive faster deceleration front"), result.frontPwm, 60);
  expectEqual(F("drive faster deceleration rear"), result.rearPwm, 60);

  result = BroonT870::updateDrivePair(state, 0, 80U, 80U, 10U, 20U,
                                      300U, 40UL, false);
  expectEqual(F("drive neutral immediately stops front"), result.frontPwm,
              0);
  expectEqual(F("drive neutral immediately stops rear"), result.rearPwm, 0);
}

void testSpeedPiController() {
  BroonT870::SpeedPiState state = {17.0f, 2.0f};
  BroonT870::resetSpeedPi(state);
  expectNear(F("PI reset integral"), state.integralPwm, 0.0f, 0.001f);
  expectNear(F("PI reset target"), state.rampedTargetKph, 0.0f, 0.001f);

  BroonT870::SpeedPiResult result = BroonT870::updateSpeedPi(
      state, 3.0f, 0.0f, 20.0f, 0.0f, 0.0f, 1.0f, 80U, 100UL);
  expectNear(F("PI target ramp"), result.rampedTargetKph, 0.1f, 0.001f);
  expectEqual(F("PI proportional output"), result.pwm, 2);

  state = {0.0f, 3.0f};
  result = BroonT870::updateSpeedPi(state, 3.0f, 0.0f, 100.0f, 0.0f,
                                    0.0f, 1.0f, 80U, 100UL);
  expectEqual(F("PI maximum clamp"), result.pwm, 80);

  state = {0.0f, 1.0f};
  result = BroonT870::updateSpeedPi(state, 1.0f, 2.0f, 20.0f, 0.0f,
                                    0.0f, 1.0f, 80U, 100UL);
  expectEqual(F("PI nonnegative output"), result.pwm, 0);

  state = {0.0f, 1.0f};
  result = BroonT870::updateSpeedPi(state, 1.0f, 0.5f, 0.0f, 20.0f,
                                    0.0f, 1.0f, 80U, 1000UL);
  expectNear(F("PI integral accumulation"), state.integralPwm, 10.0f,
             0.001f);
  expectEqual(F("PI integral output"), result.pwm, 10);

  state = {30.0f, 3.0f};
  result = BroonT870::updateSpeedPi(state, 3.0f, 0.0f, 100.0f, 20.0f,
                                    0.0f, 1.0f, 80U, 1000UL);
  expectNear(F("PI anti windup"), state.integralPwm, 30.0f, 0.001f);
  expectEqual(F("PI saturated output"), result.pwm, 80);

  result = BroonT870::updateSpeedPi(state, 0.0f, 1.0f, 20.0f, 8.0f,
                                    0.12f, 1.0f, 80U, 100UL);
  expectEqual(F("PI zero target output"), result.pwm, 0);
  expectNear(F("PI zero target reset"), state.integralPwm, 0.0f, 0.001f);
}

void testDriveFeedbackWatchdog() {
  BroonT870::DriveFeedbackWatchdogState state = {0UL, false};
  BroonT870::DriveFeedbackWatchdogResult result =
      BroonT870::updateDriveFeedbackWatchdog(
          state, 1.0f, 0.0f, 0L, 59, 60U, 1500U, 4.0f, 0UL);
  expectEqual(F("watchdog below PWM threshold"), result.noFeedbackFault, 0);
  expectEqual(F("watchdog below threshold reset"), state.timing, 0);

  result = BroonT870::updateDriveFeedbackWatchdog(
      state, 1.0f, 0.0f, 0L, 60, 60U, 1500U, 4.0f, 100UL);
  expectEqual(F("watchdog timer starts"), state.timing, 1);
  result = BroonT870::updateDriveFeedbackWatchdog(
      state, 1.0f, 0.0f, 0L, 60, 60U, 1500U, 4.0f, 1599UL);
  expectEqual(F("watchdog before timeout"), result.noFeedbackFault, 0);
  result = BroonT870::updateDriveFeedbackWatchdog(
      state, 1.0f, 0.0f, 0L, 60, 60U, 1500U, 4.0f, 1600UL);
  expectEqual(F("watchdog at timeout"), result.noFeedbackFault, 1);

  result = BroonT870::updateDriveFeedbackWatchdog(
      state, 1.0f, 0.1f, 1L, 60, 60U, 1500U, 4.0f, 1700UL);
  expectEqual(F("watchdog encoder recovery"), state.timing, 0);
  expectEqual(F("watchdog recovery no fault"), result.noFeedbackFault, 0);

  result = BroonT870::updateDriveFeedbackWatchdog(
      state, 1.0f, 4.1f, 1L, 20, 60U, 1500U, 4.0f, 1800UL);
  expectEqual(F("watchdog overspeed"), result.overspeedFault, 1);

  BroonT870::resetDriveFeedbackWatchdog(state);
  expectEqual(F("watchdog explicit reset"), state.timing, 0);
}

void testRosTargetSpeedSurvivesCommandSelection() {
  const BroonT870::VehicleCommand rc = {600, 0, 0.0f, BroonT870::MODE_RC,
                                        10UL, true, false};
  const BroonT870::VehicleCommand ros = {600, 0, 1.5f, BroonT870::MODE_ROS,
                                         20UL, true, false};
  const BroonT870::VehicleCommand selected = BroonT870::selectActiveCommand(
      rc, ros, BroonT870::MODE_ROS, true);
  expectNear(F("ROS target KPH survives selection"), selected.targetSpeedKph,
             1.5f, 0.001f);
}

void testMeasuredCalibrationAndSteeringLimits() {
  const float rpm = BroonT870::rpmFromCountDelta(10L, 300.0f, 100UL);
  const float speedMps = BroonT870::speedMpsFromRpm(rpm, 0.84823f);
  expectNear(F("encoder 10 count RPM"), rpm, 20.0f, 0.001f);
  expectNear(F("encoder calibrated KPH"), speedMps * 3.6f, 1.017876f,
             0.001f);

  expectEqual(F("normalized left endpoint"),
              BroonT870::steeringPositionPermille(1000, 1000, 512, 20),
              1000);
  expectEqual(F("normalized center"),
              BroonT870::steeringPositionPermille(512, 1000, 512, 20),
              0);
  expectEqual(F("normalized right endpoint"),
              BroonT870::steeringPositionPermille(20, 1000, 512, 20),
              -1000);
  expectEqual(F("steering reaches full PWM at 25 percent left error"),
              BroonT870::computeNormalizedSteeringPwm(
                  634, 512, 1000, 512, 20, 12U, 70U, 220U, 250U),
              -220);
  expectEqual(F("steering reaches full PWM at 25 percent right error"),
              BroonT870::computeNormalizedSteeringPwm(
                  389, 512, 1000, 512, 20, 12U, 70U, 220U, 250U),
              220);
  expectEqual(F("steering proportional below early saturation"),
              BroonT870::computeNormalizedSteeringPwm(
                  612, 512, 1000, 512, 20, 12U, 70U, 220U, 250U),
              -192);
  expectEqual(F("steering deadband"),
              BroonT870::computeNormalizedSteeringPwm(
                  523, 512, 1000, 512, 20, 12U, 70U, 220U, 250U),
              0);

  BroonT870::SteeringGuardResult guard =
      BroonT870::guardSteeringPwm(-140, 1000, 1000, 200, 100U, 140U, 60U,
                                  60U);
  expectEqual(F("steering blocks farther left"), guard.allowedPwm, 0);
  guard = BroonT870::guardSteeringPwm(140, 1000, 1000, 200, 100U, 140U, 60U,
                                      60U);
  expectEqual(F("steering limits recovery"), guard.allowedPwm, 60);
  guard = BroonT870::guardSteeringPwm(-140, 950, 1000, 200, 100U, 140U, 60U,
                                      60U);
  expectEqual(F("steering approach slowdown"), guard.allowedPwm, -70);
  guard = BroonT870::guardSteeringPwm(60, 250, 1000, 200, 100U, 140U, 60U,
                                      60U);
  expectEqual(F("steering approach preserves breakaway PWM"),
              guard.allowedPwm, 60);
  guard = BroonT870::guardSteeringPwm(60, 200, 1000, 200, 100U, 140U, 60U,
                                      60U);
  expectEqual(F("steering endpoint still blocks outward PWM"),
              guard.allowedPwm, 0);
}

void testSteeringHuntingSuppression() {
  BroonT870::SteeringStabilizerState state;
  BroonT870::resetSteeringStabilizer(state);

  int16_t pwm = BroonT870::stabilizeSteeringPwm(
      state, -63, 620, 600, 15U, 30U);
  expectEqual(F("steering reset does not suppress unsettled error"), pwm,
              -63);

  BroonT870::resetSteeringStabilizer(state);

  pwm = BroonT870::stabilizeSteeringPwm(
      state, -140, 700, 600, 15U, 30U);
  expectEqual(F("steering full response outside settle region"), pwm, -140);

  pwm = BroonT870::stabilizeSteeringPwm(state, -61, 615, 600, 15U, 30U);
  expectEqual(F("steering enters settle band"), pwm, 0);
  expectEqual(F("steering settle state active"), state.settled, 1);

  pwm = BroonT870::stabilizeSteeringPwm(state, -63, 629, 600, 15U, 30U);
  expectEqual(F("steering remains settled below restart threshold"), pwm, 0);

  pwm = BroonT870::stabilizeSteeringPwm(state, -64, 630, 600, 15U, 30U);
  expectEqual(F("steering restarts at threshold"), pwm, -64);

  pwm = BroonT870::stabilizeSteeringPwm(state, 64, 570, 600, 15U, 30U);
  expectEqual(F("steering reversal inserts neutral update"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, 64, 570, 600, 15U, 30U);
  expectEqual(F("steering reversal proceeds next update"), pwm, 64);

  BroonT870::resetSteeringStabilizer(state);
  pwm = BroonT870::stabilizeSteeringPwm(state, -140, 700, 600, 10U, 25U);
  expectEqual(F("steering begins large movement"), pwm, -140);
  pwm = BroonT870::stabilizeSteeringPwm(state, 90, 590, 600, 10U, 25U);
  expectEqual(F("small target crossing does not reverse"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, 90, 590, 600, 10U, 25U);
  expectEqual(F("small target crossing remains settled"), pwm, 0);

  BroonT870::resetSteeringStabilizer(state);
  pwm = BroonT870::stabilizeSteeringPwm(state, 0, 600, 600, 12U, 25U, 3U);
  expectEqual(F("confirmed restart test begins settled"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, -100, 650, 600, 12U, 25U,
                                        3U);
  expectEqual(F("restart excursion sample one held"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, -100, 650, 600, 12U, 25U,
                                        3U);
  expectEqual(F("restart excursion sample two held"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, -100, 650, 600, 12U, 25U,
                                        3U);
  expectEqual(F("restart excursion sample three accepted"), pwm, -100);

  pwm = BroonT870::stabilizeSteeringPwm(state, 64, 570, 600, 0U, 30U);
  expectEqual(F("steering rejects invalid settle threshold"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, 64, 570, 600, 30U, 30U);
  expectEqual(F("steering rejects overlapping thresholds"), pwm, 0);
}

void testSteeringEndpointHoldSelectionAndBehavior() {
  expectEqual(F("left endpoint target selected"),
              BroonT870::isSteeringEndpointTarget(1020, 1020, 180, 5U), 1);
  expectEqual(F("left endpoint target band selected"),
              BroonT870::isSteeringEndpointTarget(1015, 1020, 180, 5U), 1);
  expectEqual(F("left target outside endpoint band"),
              BroonT870::isSteeringEndpointTarget(1014, 1020, 180, 5U), 0);
  expectEqual(F("right endpoint target selected"),
              BroonT870::isSteeringEndpointTarget(180, 1020, 180, 5U), 1);
  expectEqual(F("right endpoint target band selected"),
              BroonT870::isSteeringEndpointTarget(185, 1020, 180, 5U), 1);
  expectEqual(F("right target outside endpoint band"),
              BroonT870::isSteeringEndpointTarget(186, 1020, 180, 5U), 0);

  BroonT870::SteeringStabilizerState state;
  BroonT870::resetSteeringStabilizer(state);
  int16_t pwm = BroonT870::stabilizeSteeringPwm(
      state, -60, 1020, 995, 25U, 60U);
  expectEqual(F("endpoint enters hold"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, -60, 1020, 973, 25U, 60U);
  expectEqual(F("endpoint ignores captured inward bounce"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, 60, 1020, 1018, 25U, 60U);
  expectEqual(F("endpoint ignores captured outward bounce"), pwm, 0);
  pwm = BroonT870::stabilizeSteeringPwm(state, -60, 1020, 960, 25U, 60U);
  expectEqual(F("endpoint reacquires after real retreat"), pwm, -60);
}

void testDisabledSteeringStallLatch() {
  BroonT870::SteeringWatchdogState state;
  BroonT870::resetSteeringWatchdog(state, 447, 0UL);

  BroonT870::SteeringWatchdogResult result =
      BroonT870::updateSteeringWatchdog(state, 447, 100, 2U, 0U, 0U, 0UL);
  expectEqual(F("disabled stall latch starts without fault"), result.stallFault,
              0);

  result = BroonT870::updateSteeringWatchdog(state, 447, 100, 2U, 0U, 0U,
                                             10000UL);
  expectEqual(F("disabled stall latch remains off without ADC movement"),
              result.stallFault, 0);
  expectEqual(F("disabled runtime latch remains off"), result.runtimeFault,
              0);
}

void testDriveFaultsLatchSafetyState() {
  BroonT870::SafetyState state = {BroonT870::STATE_ARMED,
                                  BroonT870::FAULT_NONE, 0UL, false};
  BroonT870::SafetyInputs inputs = {
      true,  true,  true,  false, true,  false, true,
      false, false, false, true,  false, BroonT870::MODE_ROS, 500U};
  BroonT870::SafetyResult result =
      BroonT870::updateSafetyState(state, inputs, 1000UL);
  expectEqual(F("no-feedback fault code"), result.activeFault,
              BroonT870::FAULT_DRIVE_NO_FEEDBACK);
  expectEqual(F("no-feedback immediate stop"), result.immediateStop, 1);

  state = {BroonT870::STATE_ARMED, BroonT870::FAULT_NONE, 0UL, false};
  inputs.driveNoFeedbackFault = false;
  inputs.driveOverspeedFault = true;
  result = BroonT870::updateSafetyState(state, inputs, 1000UL);
  expectEqual(F("overspeed fault code"), result.activeFault,
              BroonT870::FAULT_DRIVE_OVERSPEED);
  expectEqual(F("overspeed immediate stop"), result.immediateStop, 1);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  testCenterAwareRcSteeringMapping();
  testResponsiveDriveRampAndNeutralStop();
  testSpeedPiController();
  testDriveFeedbackWatchdog();
  testRosTargetSpeedSurvivesCommandSelection();
  testMeasuredCalibrationAndSteeringLimits();
  testSteeringHuntingSuppression();
  testSteeringEndpointHoldSelectionAndBehavior();
  testDisabledSteeringStallLatch();
  testDriveFaultsLatchSafetyState();
  Serial.print(F("TOTAL pass="));
  Serial.print(passCount);
  Serial.print(F(" fail="));
  Serial.println(failCount);
}

void loop() {}
