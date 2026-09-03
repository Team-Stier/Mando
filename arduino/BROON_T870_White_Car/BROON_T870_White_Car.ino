// ─────────────────────────────────────────────────────────────────────────────
// 라이브러리(도구 상자) 불러오기
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>          // 아두이노 기본 기능 (digitalWrite, analogWrite 등)
#include <util/atomic.h>      // 인터럽트 중에 데이터를 안전하게 읽기 위한 도구

#include "BroonT870Core.h"    // BROON T870 차량 전용 핵심 로직 모음

#include "BuildOptions.h"     // "ROS 켜기/끄기", "출력 허용 여부" 같은 빌드 설정 스위치
#include "ControllerConfig.h" // 핀 번호, 속도 한계값 등 구체적인 설정값들
#include "RosBridge.h"        // 컴퓨터(ROS)와 통신하는 다리 역할

// ─────────────────────────────────────────────────────────────────────────────
// using 선언: "BroonT870::ControlMode" 같은 긴 이름을 "ControlMode"로 짧게 쓰기 위한 약속
// ─────────────────────────────────────────────────────────────────────────────
using BroonT870::ControlMode;           // 조종 모드 타입 (RC 또는 ROS)
using BroonT870::ControllerState;       // 컨트롤러 상태 타입 (부팅잠금, 활성, 오류 등)
using BroonT870::DrivePairResult;       // 앞/뒤 구동 모터 계산 결과 타입
using BroonT870::DrivePairState;        // 앞/뒤 구동 모터 내부 상태 타입
using BroonT870::EncoderMeasurement;   // 엔코더 측정값 타입 (속도, 회전수 등)
using BroonT870::FaultCode;            // 오류 코드 타입
using BroonT870::MotorOutputState;     // 모터 출력 상태 타입 (방향, PWM 등)
using BroonT870::RcSnapshot;           // RC 조종기 신호 스냅샷 타입
using BroonT870::SafetyInputs;         // 안전 판단에 필요한 입력값 묶음 타입
using BroonT870::SafetyResult;         // 안전 판단 결과 타입
using BroonT870::SafetyState;          // 안전 시스템 내부 상태 타입
using BroonT870::SpeedPiState;
using BroonT870::SteeringWatchdogState; // 조향 워치독(감시자) 내부 상태 타입
using BroonT870::VehicleCommand;       // 차량 명령 타입 (속도, 조향, 유효 여부 등)
using BroonT870::MODE_RC;              // RC 조종 모드를 나타내는 상수
using BroonT870::MODE_ROS;             // ROS 자율주행 모드를 나타내는 상수
using BroonT870::STATE_BOOT_LOCKED;    // 부팅 직후 잠금 상태 상수 (출력 금지)
using BroonT870::STATE_FAULT_LATCHED;  // 오류 발생 후 잠금 상태 상수
using BroonT870::FAULT_CONFIGURATION;  // 설정값 오류 코드 상수
using BroonT870::FAULT_NONE;           // 오류 없음을 나타내는 상수
using BroonT870::FAULT_STEERING_RUNTIME; // 조향 중 실행 오류 코드 상수
using BroonT870::FAULT_STEERING_SENSOR;  // 조향 센서 오류 코드 상수
using BroonT870::FAULT_STEERING_STALL;   // 조향 모터 막힘 오류 코드 상수
using BroonT870::FAULT_DRIVE_NO_FEEDBACK;
using BroonT870::FAULT_DRIVE_OVERSPEED;

using BroonT870Controller::MotorPins;  // 모터 핀 묶음 타입 (PWM핀 + 방향핀)

// ─────────────────────────────────────────────────────────────────────────────
// namespace {}: 이 안의 변수/함수는 이 파일 안에서만 사용 가능 (이름 충돌 방지)
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// 앞바퀴 엔코더(바퀴 회전 센서) 총 카운트 — volatile: 인터럽트가 언제든 바꿀 수 있어 항상 최신값 읽기
volatile long frontEncoderCount = 0;
// 직전 엔코더 상태값 (A핀/B핀 조합) — 현재 상태와 비교해 회전 방향 판단
volatile uint8_t previousFrontEncoderState = 0U;

BroonT870::BroonT870RcInput rcInput; // RC 조종기 신호를 수신/파싱하는 객체
#if BROON_ENABLE_ROS                 // ROS 기능이 켜져 있을 때만 컴파일
BroonT870Controller::RosBridge rosBridge; // 컴퓨터(ROS)와 통신하는 브리지 객체
#endif
ControlMode selectedMode = MODE_RC;  // 현재 선택된 조종 모드 (기본값: RC 모드)
VehicleCommand rcCommand  = {0, 0, 0.0f, MODE_RC,  0UL, false, false};
VehicleCommand rosCommand = {0, 0, 0.0f, MODE_ROS, 0UL, false, false};
VehicleCommand activeCommand = {0, 0, 0.0f, MODE_RC, 0UL, false, false};
SafetyState safetyState   = {STATE_BOOT_LOCKED, FAULT_NONE, 0UL, false}; // 안전 시스템 내부 상태 (부팅 잠금으로 시작)
SafetyResult safetyResult = {STATE_BOOT_LOCKED, FAULT_NONE, false, true}; // 안전 판단 결과 (출력 금지로 시작)
DrivePairState drivePairState;                                  // 앞/뒤 구동 모터 내부 상태 (방향 전환 타이밍 추적 등)
MotorOutputState frontMotorOutputState    = {false};            // 앞 구동 모터 현재 출력 상태
MotorOutputState rearMotorOutputState     = {false};            // 뒤 구동 모터 현재 출력 상태
MotorOutputState steeringMotorOutputState = {false};            // 조향(핸들) 모터 현재 출력 상태
EncoderMeasurement frontEncoderMeasurement = {0, 0, 0.0f, 0.0f, false}; // 앞바퀴 엔코더 측정값 {변화카운트, 총카운트, RPM, 속도m/s, 보정됨?}
SpeedPiState rosSpeedPiState = {0.0f, 0.0f};
BroonT870::DriveFeedbackWatchdogState driveFeedbackWatchdogState = {0UL,
                                                                    false};
RcSnapshot latestRcSnapshot = {0U, 0U, 0U, false, false, false}; // 가장 최근 RC 원시 펄스와 유효성

bool outputHardwareInitialized = false; // 모터 핀 초기화가 완료됐는지 여부
bool outputsAllowed = false;            // 현재 모터 출력이 허용된 상태인지 여부
bool configurationValid = false;        // 설정값(캘리브레이션 등)이 모두 유효한지 여부
bool steeringSensorFault = false;       // 조향 센서 오류 발생 여부
bool steeringStallFault = false;        // 조향 모터 막힘(스톨) 오류 발생 여부
bool steeringRuntimeFault = false;      // 조향 실행 중 오류 발생 여부
bool driveNoFeedbackFault = false;
bool driveOverspeedFault = false;
bool rcStopActive = true;               // 조종기 throttle-cut 또는 신호 손실 정지 상태(초기값: 정지)
int16_t rosSpeedControlPwm = 0;
int16_t latestDriveRequestedPwm = 0;
int16_t latestFrontDrivePwm = 0;
int16_t latestRearDrivePwm = 0;
int16_t latestSteeringGuardedPwm = 0;
int16_t latestSteeringAuthorizedPwm = 0;
float measuredAbsoluteKph = 0.0f;
int16_t latestSteeringAdc = 0;          // 조향 센서 최신 ADC값 (0~1023, 현재 핸들 각도)
int16_t minimumSeenSteeringAdc = 1023;  // 부팅 후 관측한 조향 ADC 최솟값
int16_t maximumSeenSteeringAdc = 0;     // 부팅 후 관측한 조향 ADC 최댓값
SteeringWatchdogState steeringWatchdogState = {false, 0, 0UL, 0UL, 0}; // 조향 워치독 내부 상태
BroonT870::SteeringStabilizerState steeringStabilizerState =
    BroonT870::initialSteeringStabilizerState();
uint32_t lastControlMs = 0UL;           // 마지막으로 제어 주기 처리를 실행한 시각(ms)
uint32_t lastEncoderSampleMs = 0UL;    // 마지막으로 엔코더를 샘플링한 시각(ms)
uint32_t lastStatusMs = 0UL;           // 마지막으로 상태를 출력한 시각(ms)
uint32_t latestRcReadUs = 0UL;         // A0/A1/A2 pulseIn 3채널을 읽는 데 걸린 시간
bool controlClockStarted = false;       // 제어 주기 타이머가 시작됐는지 여부
bool controlTickDue = false;            // 이번 루프에서 제어 처리를 해야 하는지 여부
bool encoderClockStarted = false;       // 엔코더 샘플링 타이머가 시작됐는지 여부
uint32_t latestEncoderSampleTimeMs = 0UL;
bool statusClockStarted = false;        // 상태 출력 타이머가 시작됐는지 여부

void resetRosSpeedControl();

// Reject a one-sample potentiometer spike without adding time-domain lag.
int16_t readSteeringAdcMedian3() {
  const int16_t a = analogRead(BroonT870Controller::kSteeringSensorPin);
  const int16_t b = analogRead(BroonT870Controller::kSteeringSensorPin);
  const int16_t c = analogRead(BroonT870Controller::kSteeringSensorPin);
  int16_t instantMedian = 0;
  if (a > b) {
    instantMedian = b > c ? b : (a > c ? c : a);
  } else {
    instantMedian = a > c ? a : (b > c ? c : b);
  }

  static bool initialized = false;
  static int16_t history[3] = {0, 0, 0};
  static uint8_t nextIndex = 0U;
  if (!initialized) {
    history[0] = history[1] = history[2] = instantMedian;
    initialized = true;
  } else {
    history[nextIndex] = instantMedian;
    nextIndex = static_cast<uint8_t>((nextIndex + 1U) % 3U);
  }

  const int16_t x = history[0];
  const int16_t y = history[1];
  const int16_t z = history[2];
  if (x > y) {
    if (y > z) return y;
    return x > z ? z : x;
  }
  if (x > z) return x;
  return y > z ? z : y;
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 조향 캘리브레이션(보정값)이 물리적으로 유효한지 확인
// ─────────────────────────────────────────────────────────────────────────────
bool isValidSteeringCalibration() {
  const BroonT870Controller::SteeringCalibration& calibration =
      BroonT870Controller::kSteeringCalibration; // 설정 파일에 저장된 조향 보정값 가져오기
  return BroonT870::isSteeringCalibrationValid(
      calibration.leftMechanicalAdc,  // 왼쪽 기계적 한계(절대 최대) ADC값
      calibration.leftSafeAdc,        // 왼쪽 안전 한계 ADC값
      calibration.centerAdc,          // 정중앙 ADC값
      calibration.rightSafeAdc,       // 오른쪽 안전 한계 ADC값
      calibration.rightMechanicalAdc, // 오른쪽 기계적 한계(절대 최대) ADC값
      calibration.adcIncreasesRight); // ADC값이 오른쪽으로 갈수록 커지는지 방향 정보
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 모든 설정값이 올바른지 종합 검증 — 하나라도 false면 차가 출발하지 않음
// ─────────────────────────────────────────────────────────────────────────────
bool validateConfiguration() {
  return BROON_STEERING_CALIBRATION_CONFIRMED &&         // 조향 보정 확인 플래그가 설정됐는지
         BroonT870::isRcChannelCalibrationValid(
             BroonT870Controller::kRcSteerCalibration) && // RC 조향 채널 보정값이 올바른지
         BroonT870::isRcThrottleConfigurationValid(
             BroonT870Controller::kRcThrottleCalibration,
             BroonT870Controller::kRcThrottleDeadbandUs) && // RC 스로틀 보정값과 데드밴드가 올바른지
         BroonT870::isAuxModeConfigurationValid(
             BroonT870Controller::kRcAuxCalibration,
             BroonT870Controller::kAuxRosThresholdUs,
             BroonT870Controller::kAuxRcThresholdUs) && // RC AUX 채널(모드 전환 레버) 설정이 올바른지
         BroonT870Controller::countsPerWheelRev > 0.0f && // 바퀴 한 바퀴당 엔코더 카운트가 양수인지
         BroonT870Controller::wheelCircumferenceM > 0.0f && // 바퀴 둘레가 양수인지
         isValidSteeringCalibration() &&                  // 조향 캘리브레이션이 유효한지
         BroonT870Controller::kSteeringMaximumPwm > 0U && // 조향 최대 PWM이 양수인지
         BroonT870Controller::kSteeringFullPwmErrorPermille > 0U &&
         BroonT870Controller::kSteeringFullPwmErrorPermille <= 1000U &&
         BroonT870Controller::kSteeringRestartConfirmSamples > 0U &&
         BroonT870Controller::kSteeringEndpointSettleErrorAdc <
             BroonT870Controller::kSteeringEndpointRestartErrorAdc &&
         BroonT870Controller::kDriveForwardMaxPwm > 0U && // 전진 최대 PWM이 양수인지
         BroonT870Controller::kDriveReverseMaxPwm > 0U;   // 후진 최대 PWM이 양수인지
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 앞바퀴 엔코더 A·B 두 핀의 현재 HIGH/LOW 상태를 읽어 0~3 숫자로 반환
// ─────────────────────────────────────────────────────────────────────────────
uint8_t readFrontEncoderState() {
  return (digitalRead(BroonT870Controller::kFrontEncoderAPin) == HIGH ? 1U  // A핀이 HIGH면 비트0=1
                                                                        : 0U) |
         (digitalRead(BroonT870Controller::kFrontEncoderBPin) == HIGH ? 2U  // B핀이 HIGH면 비트1=1
                                                                        : 0U);
  // 결과: A=LOW,B=LOW → 0 / A=HIGH,B=LOW → 1 / A=LOW,B=HIGH → 2 / A=HIGH,B=HIGH → 3
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 이전 엔코더 상태와 현재 상태를 비교해 카운트를 +1 또는 -1 업데이트
// ─────────────────────────────────────────────────────────────────────────────
void updateFrontEncoderCount() {
  const uint8_t currentState = readFrontEncoderState(); // 현재 엔코더 핀 상태 읽기
  frontEncoderCount +=
      BroonT870::quadratureStep(previousFrontEncoderState, currentState); // 이전→현재 전이로 +1/-1/0 결정
  previousFrontEncoderState = currentState; // 현재 상태를 다음 비교를 위해 저장
}

// ─────────────────────────────────────────────────────────────────────────────
// ISR (인터럽트 서비스 루틴): 엔코더 A핀 신호가 바뀌는 순간 자동 호출
// ─────────────────────────────────────────────────────────────────────────────
void frontEncoderAIsr() {
  updateFrontEncoderCount(); // 카운트 업데이트
}

// ISR: 엔코더 B핀 신호가 바뀌는 순간 자동 호출
void frontEncoderBIsr() {
  updateFrontEncoderCount(); // 카운트 업데이트
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 인터럽트에 방해받지 않고 엔코더 카운트를 안전하게 읽어 반환
// ─────────────────────────────────────────────────────────────────────────────
long atomicFrontEncoderCount() {
  long result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // 이 블록 안에서는 인터럽트가 발생하지 않음 (안전하게 읽기)
    result = frontEncoderCount;       // 엔코더 카운트 읽기
  }
  return result; // 읽은 값 반환
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 지정한 모터 핀에 방향과 PWM 신호를 출력
//   pins          : 해당 모터의 PWM핀 + 방향핀 묶음
//   outputState   : 이 모터의 이전 출력 상태 (방향 변화 추적용)
//   logicalPwm    : 요청 PWM값 (양수=전진, 음수=후진, 0=정지)
//   invertDirection: 모터가 반대로 연결됐을 때 방향 반전 플래그
// ─────────────────────────────────────────────────────────────────────────────
void writeMotor(const MotorPins& pins, MotorOutputState& outputState,
                int16_t logicalPwm,
                bool invertDirection) {
  const bool authorized = outputHardwareInitialized &&     // 하드웨어 초기화 완료?
                          BROON_ENABLE_ACTUATOR_OUTPUTS && // 빌드 설정에서 출력 허용?
                          outputsAllowed;                  // 안전 시스템에서 출력 허용?
  const BroonT870::MotorOutputPlan plan = BroonT870::planMotorOutput(
      outputState, logicalPwm, invertDirection, authorized); // 실제 출력할 방향·PWM 계획 수립
  analogWrite(pins.pwm, 0);         // 일단 PWM을 0으로 초기화 (글리치 방지)
  if (plan.pwm == 0U) {
    return;                         // 출력값이 0이면 여기서 종료 (정지 상태 유지)
  }
  if (plan.directionWriteRequired) {                              // 방향 핀을 바꿔야 하면
    digitalWrite(pins.dir, plan.directionHigh ? HIGH : LOW);     // 방향 핀에 HIGH/LOW 출력
  }
  analogWrite(pins.pwm, plan.pwm);  // 계산된 PWM 값을 속도 핀에 출력
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 비상 상황 시 모든 모터(앞구동/뒤구동/조향)를 즉시 정지
// ─────────────────────────────────────────────────────────────────────────────
void forceAllOutputsOff() {
  analogWrite(BroonT870Controller::kFrontDrivePins.pwm, 0); // 앞 구동 모터 PWM → 0 (정지)
  analogWrite(BroonT870Controller::kRearDrivePins.pwm, 0);  // 뒤 구동 모터 PWM → 0 (정지)
  analogWrite(BroonT870Controller::kSteeringPins.pwm, 0);   // 조향 모터 PWM → 0 (정지)
  latestSteeringGuardedPwm = 0;
  latestSteeringAuthorizedPwm = 0;
  BroonT870::resetSteeringStabilizer(steeringStabilizerState);
  latestFrontDrivePwm = 0;
  latestRearDrivePwm = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 조향 오류를 기록하고 시스템을 잠금 상태로 전환 (재시작 전까지 복구 불가)
//   fault : 발생한 오류 코드
//   nowMs : 현재 시각(ms)
// ─────────────────────────────────────────────────────────────────────────────
void latchControllerFault(FaultCode fault, uint32_t nowMs) {
  if (safetyState.latchedFault == FAULT_NONE) {
    safetyState.latchedFault = fault; // 첫 번째 오류만 기록 (나중 오류로 덮어쓰지 않음)
  }
  safetyState.state = STATE_FAULT_LATCHED; // 상태를 "오류 잠금"으로 변경
  outputsAllowed = false;                  // 모든 모터 출력 즉시 금지
  BroonT870::resetSteeringWatchdog(steeringWatchdogState,
                                   latestSteeringAdc, nowMs); // 조향 워치독 초기화
  BroonT870::resetSpeedPi(rosSpeedPiState);
  BroonT870::resetDriveFeedbackWatchdog(driveFeedbackWatchdogState);
  rosSpeedControlPwm = 0;
  forceAllOutputsOff();                    // 모든 모터 즉시 정지
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 설정된 제어 주기가 됐는지 확인 — true 반환 시 이번 주기에 제어 처리 실행
// ─────────────────────────────────────────────────────────────────────────────
bool controlPeriodDue(uint32_t nowMs) {
  if (!controlClockStarted ||                       // 타이머가 아직 시작 안 됐거나
      BroonT870::hasElapsed(nowMs, lastControlMs,
                            BroonT870Controller::kControlIntervalMs)) { // 제어 주기(ms)가 경과했으면
    lastControlMs = nowMs;      // 마지막 처리 시각을 현재로 업데이트
    controlClockStarted = true; // 타이머 시작됨 표시
    return true;                // "이번 주기 처리할 시간이다!"
  }
  return false; // "아직 주기가 안 됐다"
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 모든 모터 핀을 OUTPUT으로 설정하고 초기값(0) 지정
// ─────────────────────────────────────────────────────────────────────────────
void configureMotorPins() {
  pinMode(BroonT870Controller::kFrontDrivePins.pwm, OUTPUT); // 앞 구동 모터 속도(PWM) 핀 → 출력 모드
  pinMode(BroonT870Controller::kRearDrivePins.pwm,  OUTPUT); // 뒤 구동 모터 속도(PWM) 핀 → 출력 모드
  pinMode(BroonT870Controller::kSteeringPins.pwm,   OUTPUT); // 조향 모터 속도(PWM) 핀 → 출력 모드
  forceAllOutputsOff();                                      // 초기화 직후 모든 모터를 0으로 안전하게 시작

  pinMode(BroonT870Controller::kFrontDrivePins.dir, OUTPUT); // 앞 구동 모터 방향 핀 → 출력 모드
  pinMode(BroonT870Controller::kRearDrivePins.dir,  OUTPUT); // 뒤 구동 모터 방향 핀 → 출력 모드
  pinMode(BroonT870Controller::kSteeringPins.dir,   OUTPUT); // 조향 모터 방향 핀 → 출력 모드
  digitalWrite(BroonT870Controller::kFrontDrivePins.dir, LOW); // 앞 구동 방향 핀 초기값 LOW
  digitalWrite(BroonT870Controller::kRearDrivePins.dir,  LOW); // 뒤 구동 방향 핀 초기값 LOW
  digitalWrite(BroonT870Controller::kSteeringPins.dir,   LOW); // 조향 방향 핀 초기값 LOW
  frontMotorOutputState.directionHigh    = false; // 앞 모터 방향 상태 초기화 (LOW)
  rearMotorOutputState.directionHigh     = false; // 뒤 모터 방향 상태 초기화 (LOW)
  steeringMotorOutputState.directionHigh = false; // 조향 모터 방향 상태 초기화 (LOW)
  outputHardwareInitialized = true;               // 하드웨어 초기화 완료 표시
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 컴퓨터(ROS)에서 최신 차량 명령을 받아 rosCommand에 업데이트
// ─────────────────────────────────────────────────────────────────────────────
void updateRosCommand(uint32_t nowMs) {
#if BROON_ENABLE_ROS // ROS 기능이 켜진 경우에만 이 코드 포함
  rosCommand = rosBridge.vehicleCommand(
      nowMs,                                             // 현재 시각(ms)
      BroonT870Controller::kRosTimeoutMs,               // ROS 명령 유효 시간 — 이보다 오래됐으면 무시
      BroonT870Controller::kRosMaximumForwardKph,       // ROS 전진 최대 속도(km/h)
      BroonT870Controller::kRosMinimumDegrees,          // ROS 조향 최소 각도
      BroonT870Controller::kRosMaximumDegrees,          // ROS 조향 최대 각도
      BroonT870Controller::kSteeringCalibration.leftSafeAdc,  // 왼쪽 안전 ADC값
      BroonT870Controller::kSteeringCalibration.centerAdc,    // 중앙 ADC값
      BroonT870Controller::kSteeringCalibration.rightSafeAdc); // 오른쪽 안전 ADC값
#else
  (void)nowMs; // ROS 꺼져있으면 아무것도 안 함 (nowMs 미사용 경고 억제)
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: RC 조종기 신호를 읽고, 모드 전환 및 최종 실행 명령(activeCommand)을 결정
// ─────────────────────────────────────────────────────────────────────────────
void updateInputCommands(uint32_t nowMs, uint32_t nowUs) {
  const uint32_t rcReadStartedUs = micros();
  latestRcSnapshot =
      rcInput.snapshot(nowUs, BroonT870Controller::kRcTimeoutMs); // RC 수신기 신호를 현시각 기준으로 읽기
  latestRcReadUs = micros() - rcReadStartedUs;
  const RcSnapshot& snapshot = latestRcSnapshot;
  rcStopActive = !snapshot.throttleValid ||
                 snapshot.throttlePulseUs <
                     BroonT870Controller::kRcStopThresholdUs;
  const bool auxValid =
      snapshot.auxValid && BroonT870::isRcPulseWithinCalibration(
                               snapshot.auxPulseUs,
                               BroonT870Controller::kRcAuxCalibration); // AUX 채널 신호가 유효하고 보정 범위 내인지
  if (auxValid) {
    selectedMode = BroonT870::updateModeFromAux(
        selectedMode, snapshot.auxPulseUs,
        BroonT870Controller::kAuxRosThresholdUs,  // 이 값 이하면 ROS 모드로 전환
        BroonT870Controller::kAuxRcThresholdUs);  // 이 값 이상이면 RC 모드로 전환
  }

  const bool rcValid = !rcStopActive &&         // throttle-cut이 해제되어 있고
                        snapshot.steerValid &&   // 조향 채널 신호가 유효하고
                        snapshot.throttleValid && // 스로틀 채널 신호가 유효하고
                        auxValid;                 // AUX 채널 신호가 유효해야 RC 유효
  if (rcValid) {
    rcCommand.drivePwm = BroonT870::rcThrottleToSignedPwm(
        snapshot.throttlePulseUs,                              // RC 스로틀 신호 펄스폭(μs)
        BroonT870Controller::kRcThrottleCalibration.minimumUs, // 스로틀 최솟값(μs)
        BroonT870Controller::kRcThrottleCalibration.centerUs,  // 스로틀 중립값(μs)
        BroonT870Controller::kRcThrottleCalibration.maximumUs, // 스로틀 최댓값(μs)
        BroonT870Controller::kRcThrottleDeadbandUs,            // 중립 주변 데드밴드(μs)
        BroonT870Controller::kDriveForwardMaxPwm,              // 전진 최대 PWM
        BroonT870Controller::kDriveReverseMaxPwm);             // 후진 최대 PWM → 부호 있는 PWM값 반환
    rcCommand.steerTargetAdc = BroonT870::rcSteerToTargetAdc(
        snapshot.steerPulseUs,                                 // RC 조향 신호 펄스폭(μs)
        BroonT870Controller::kRcSteerCalibration.minimumUs,    // 조향 채널 최솟값(μs)
        BroonT870Controller::kRcSteerCalibration.centerUs,     // 조향 채널 중앙값(μs)
        BroonT870Controller::kRcSteerCalibration.maximumUs,    // 조향 채널 최댓값(μs)
        BroonT870Controller::kSteeringCalibration.leftSafeAdc, // 왼쪽 안전 ADC값
        BroonT870Controller::kSteeringCalibration.centerAdc,   // 실제 직진 ADC값
        BroonT870Controller::kSteeringCalibration.rightSafeAdc); // 오른쪽 안전 ADC값 → 목표 ADC값 반환
    rcCommand.targetSpeedKph = 0.0f;
    rcCommand.receivedAtMs = nowMs;   // 명령 수신 시각 기록
    rcCommand.valid = true;           // RC 명령 유효 표시
    rcCommand.brakeRequested = false; // 브레이크 요청 없음
  } else {
    rcCommand = {0, 0, 0.0f, MODE_RC, nowMs, false, false}; // RC 신호 없으면 명령 초기화(무효)
  }

  activeCommand = BroonT870::selectActiveCommand(rcCommand, rosCommand,
                                                   selectedMode,
                                                   auxValid); // 현재 모드에 맞는 명령(RC 또는 ROS) 선택
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 안전 시스템 상태를 업데이트하고 출력 허용 여부를 결정
// ─────────────────────────────────────────────────────────────────────────────
void updateSafety(uint32_t nowMs) {
  controlTickDue = controlPeriodDue(nowMs); // 제어 주기가 됐는지 확인
  if (!controlTickDue) {
    return; // 주기가 아니면 아무것도 안 함
  }

  const bool throttleNeutral = activeCommand.valid &&
      (selectedMode == MODE_ROS
           ? activeCommand.targetSpeedKph <= 0.0f
           : BroonT870::signOf(activeCommand.drivePwm) == 0);
  const SafetyInputs inputs = {
      BROON_ENABLE_ACTUATOR_OUTPUTS != 0,         // 빌드 설정에서 출력 허용?
      BROON_STEERING_CALIBRATION_CONFIRMED != 0,  // 조향 보정 확인됨?
      configurationValid,                         // 설정값 전체 유효?
      rcStopActive,                               // 조종기 throttle-cut 또는 신호 손실 정지?
      activeCommand.valid,                        // 실행 명령이 유효?
      activeCommand.brakeRequested,               // 브레이크 요청됨?
      throttleNeutral,                            // 스로틀이 중립?
      steeringSensorFault,                        // 조향 센서 오류?
      steeringStallFault,                         // 조향 스톨 오류?
      steeringRuntimeFault,                       // 조향 런타임 오류?
      driveNoFeedbackFault,                       // 구동 명령 중 엔코더 무응답?
      driveOverspeedFault,                        // 측정 속도 과속?
      selectedMode,                               // 현재 선택된 모드(RC/ROS)
      BroonT870Controller::kNeutralHoldMs,        // 출발 전 중립 유지 시간 설정
  };
  safetyResult = BroonT870::updateSafetyState(safetyState, inputs, nowMs); // 모든 입력을 종합해 안전 판단
  outputsAllowed = safetyResult.outputsAllowed; // 판단 결과에 따라 출력 허용 여부 업데이트
  if (safetyResult.immediateStop || !outputsAllowed) { // 즉시 정지 명령이거나 출력 금지면
    if (safetyResult.state == STATE_FAULT_LATCHED) resetRosSpeedControl();
    forceAllOutputsOff(); // 모든 모터 즉시 정지
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 조향(핸들) 모터를 목표 각도로 제어
// ─────────────────────────────────────────────────────────────────────────────
void updateSteering(uint32_t nowMs) {
  if (!controlTickDue) {
    return; // 제어 주기가 아니면 아무것도 안 함
  }

  // 이 주기의 모든 안전 판단은 아래 한 번의 ADC 읽기 값으로 수행
  latestSteeringAdc = readSteeringAdcMedian3(); // 지연 없이 A3 순간 스파이크 제거
  if (latestSteeringAdc < minimumSeenSteeringAdc) {
    minimumSeenSteeringAdc = latestSteeringAdc;
  }
  if (latestSteeringAdc > maximumSeenSteeringAdc) {
    maximumSeenSteeringAdc = latestSteeringAdc;
  }
  if (latestSteeringAdc < BroonT870Controller::kSteeringElectricalMinAdc || // ADC값이 최솟값보다 작거나
      latestSteeringAdc > BroonT870Controller::kSteeringElectricalMaxAdc) { // 최댓값보다 크면 → 센서 단선/단락 의심
    steeringSensorFault = true;
    latchControllerFault(FAULT_STEERING_SENSOR, nowMs); // 센서 오류로 잠금 처리
    return;
  }

  const int16_t requestedPwm = BroonT870::computeNormalizedSteeringPwm(
      activeCommand.steerTargetAdc,                          // 목표 조향 ADC값
      latestSteeringAdc,                                     // 현재 조향 ADC값
      BroonT870Controller::kSteeringCalibration.leftSafeAdc, // 왼쪽을 정규화 위치 +1000으로 사용
      BroonT870Controller::kSteeringCalibration.centerAdc,   // 직진을 정규화 위치 0으로 사용
      BroonT870Controller::kSteeringCalibration.rightSafeAdc,// 오른쪽을 정규화 위치 -1000으로 사용
      BroonT870Controller::kSteeringDeadbandAdc,             // 이 오차 이내면 모터 구동 안 함(데드밴드)
      BroonT870Controller::kSteeringMinimumPwm,              // 출력 하한값 (너무 약한 PWM은 올림)
      BroonT870Controller::kSteeringMaximumPwm,              // 출력 상한값
      BroonT870Controller::kSteeringFullPwmErrorPermille);   // 전체 조향 범위 중 최대 PWM에 도달할 오차 비율

  const bool endpointTarget = BroonT870::isSteeringEndpointTarget(
      activeCommand.steerTargetAdc,
      BroonT870Controller::kSteeringCalibration.leftSafeAdc,
      BroonT870Controller::kSteeringCalibration.rightSafeAdc,
      BroonT870Controller::kSteeringEndpointTargetBandAdc);
  const uint16_t settleErrorAdc =
      endpointTarget ? BroonT870Controller::kSteeringEndpointSettleErrorAdc
                     : BroonT870Controller::kSteeringSettleErrorAdc;
  const uint16_t restartErrorAdc =
      endpointTarget ? BroonT870Controller::kSteeringEndpointRestartErrorAdc
                     : BroonT870Controller::kSteeringRestartErrorAdc;
  const int16_t stabilizedPwm = BroonT870::stabilizeSteeringPwm(
      steeringStabilizerState, requestedPwm,
      activeCommand.steerTargetAdc, latestSteeringAdc,
      settleErrorAdc, restartErrorAdc,
      BroonT870Controller::kSteeringRestartConfirmSamples);
  const BroonT870::SteeringGuardResult guarded = BroonT870::guardSteeringPwm(
      stabilizedPwm, latestSteeringAdc,
      BroonT870Controller::kSteeringCalibration.leftSafeAdc,    // 왼쪽 안전 한계 ADC값
      BroonT870Controller::kSteeringCalibration.rightSafeAdc,   // 오른쪽 안전 한계 ADC값
      BroonT870Controller::kSteeringApproachBandAdc,            // 안전 한계 접근 구간(이 범위 내면 속도 줄임)
      BroonT870Controller::kSteeringMaximumPwm,                 // 최대 PWM
      BroonT870Controller::kSteeringMinimumPwm,                 // 감속 후에도 유지할 최소 유효 PWM
      BroonT870Controller::kSteeringRecoveryPwm);               // 한계 벗어날 때 복귀 속도 → 가드 적용 PWM 반환
  if (guarded.sensorFault) {          // 가드 계산 중 센서 오류 감지되면
    steeringSensorFault = true;
    latchControllerFault(FAULT_STEERING_SENSOR, nowMs);
    return;
  }
  if (guarded.configurationFault) {   // 가드 계산 중 설정 오류 감지되면
    latchControllerFault(FAULT_CONFIGURATION, nowMs);
    return;
  }

  const int16_t guardedPwm = guarded.allowedPwm; // 가드를 통과한 최종 PWM값
  const bool steeringOutputAuthorized =
      outputHardwareInitialized &&      // 하드웨어 초기화 완료?
      BROON_ENABLE_ACTUATOR_OUTPUTS &&  // 빌드 설정에서 출력 허용?
      outputsAllowed &&                 // 안전 시스템에서 출력 허용?
      !safetyResult.immediateStop;      // 즉시 정지 명령 없음?
  const int16_t authorizedPwm = steeringOutputAuthorized ? guardedPwm : 0; // 허가됐으면 guardedPwm, 아니면 0
  latestSteeringGuardedPwm = guardedPwm;
  latestSteeringAuthorizedPwm = authorizedPwm;
  const BroonT870::SteeringWatchdogResult watchdog =
      BroonT870::updateSteeringWatchdog(
          steeringWatchdogState, latestSteeringAdc, authorizedPwm,
          BroonT870Controller::kSteeringProgressDeltaAdc, // 이만큼 변해야 "움직이고 있다"고 인정
          BroonT870Controller::kSteeringNoProgressMs,     // 이 시간 동안 움직임 없으면 스톨 오류
          BroonT870Controller::kSteeringMaxDriveMs,       // 최대 연속 구동 시간(0이면 fault 4 비활성)
          nowMs);                                         // → 워치독 결과 반환
  if (watchdog.stallFault) {           // 모터가 막혀서 못 움직이는 스톨 감지되면
    steeringStallFault = true;
    latchControllerFault(FAULT_STEERING_STALL, nowMs);
    return;
  }
  if (watchdog.runtimeFault) {         // 너무 오래 구동한 런타임 오류 감지되면
    steeringRuntimeFault = true;
    latchControllerFault(FAULT_STEERING_RUNTIME, nowMs);
    return;
  }

  writeMotor(BroonT870Controller::kSteeringPins, steeringMotorOutputState,
             guardedPwm,                                        // 가드를 통과한 PWM값 출력
             BroonT870Controller::kSteeringDirectionInverted);  // 필요시 방향 반전
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 앞/뒤 구동 모터에 가속/감속 제어를 적용하여 PWM 출력
// ─────────────────────────────────────────────────────────────────────────────
void updateDrive(uint32_t nowMs) {
  if (!controlTickDue) {
    return; // 제어 주기가 아니면 아무것도 안 함
  }
  int16_t requestedDrivePwm =
      selectedMode == MODE_ROS ? rosSpeedControlPwm : activeCommand.drivePwm;
  if ((requestedDrivePwm > 0 &&
       measuredAbsoluteKph >= BroonT870Controller::kDriveForwardSpeedLimitKph) ||
      (requestedDrivePwm < 0 &&
       measuredAbsoluteKph >= BroonT870Controller::kDriveReverseSpeedLimitKph)) {
    requestedDrivePwm = 0;
  }
  latestDriveRequestedPwm = requestedDrivePwm;
  const DrivePairResult pair = BroonT870::updateDrivePair(
      drivePairState,                                   // 앞/뒤 구동 내부 상태
      requestedDrivePwm,                                // RC PWM 또는 ROS PI 결과
      BroonT870Controller::kDriveForwardMaxPwm,         // 전진 최대 PWM 한계
      BroonT870Controller::kDriveReverseMaxPwm,         // 후진 최대 PWM 한계
      BroonT870Controller::kDriveAccelerationRampStep,  // 가속 시 한 주기당 PWM 변화량
      BroonT870Controller::kDriveDecelerationRampStep,  // 감속 시 한 주기당 PWM 변화량
      BroonT870Controller::kDirectionInterlockMs,       // 방향 전환 시 정지 대기 시간(기계 충격 방지)
      nowMs,                                            // 현재 시각
      safetyResult.immediateStop || !outputsAllowed);   // 즉시 정지 필요? → 앞/뒤 모터 PWM 계산 결과 반환
  latestFrontDrivePwm = pair.frontPwm;
  latestRearDrivePwm = pair.rearPwm;
  writeMotor(BroonT870Controller::kFrontDrivePins, frontMotorOutputState,
             pair.frontPwm,                                       // 계산된 앞 모터 PWM
             BroonT870Controller::kFrontDriveDirectionInverted);  // 필요시 방향 반전
  writeMotor(BroonT870Controller::kRearDrivePins, rearMotorOutputState,
             pair.rearPwm,                                        // 계산된 뒤 모터 PWM
             BroonT870Controller::kRearDriveDirectionInverted);   // 필요시 방향 반전
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 앞바퀴 엔코더로 RPM과 속도(m/s)를 주기적으로 계산
// ─────────────────────────────────────────────────────────────────────────────
bool updateFrontEncoderMeasurement(uint32_t nowMs) {
  if (!encoderClockStarted) {
    lastEncoderSampleMs = nowMs; // 첫 실행 시 기준 시각 설정
    encoderClockStarted = true;
    return false;
  }
  if (!BroonT870::hasElapsed(nowMs, lastEncoderSampleMs,
                              BroonT870Controller::kEncoderSampleIntervalMs)) {
    return false; // 샘플링 주기가 아직 안 됐으면 패스
  }
  const uint32_t sampleTimeMs = nowMs - lastEncoderSampleMs; // 이번 샘플링 간격(ms)
  const long currentCount = atomicFrontEncoderCount();        // 현재 엔코더 총 카운트 (안전하게 읽기)
  frontEncoderMeasurement.deltaCount =
      currentCount - frontEncoderMeasurement.totalCount;      // 이번 주기에 증가한 카운트
  frontEncoderMeasurement.totalCount = currentCount;          // 총 카운트 업데이트
  frontEncoderMeasurement.rpm = BroonT870::rpmFromCountDelta(
      frontEncoderMeasurement.deltaCount,                     // 변화 카운트
      BroonT870Controller::countsPerWheelRev,                 // 바퀴 한 바퀴당 카운트 수
      sampleTimeMs);                                          // 측정 시간 → RPM 계산
  frontEncoderMeasurement.speedMps = BroonT870::speedMpsFromRpm(
      frontEncoderMeasurement.rpm,                            // 분당 회전수(RPM)
      BroonT870Controller::wheelCircumferenceM);              // 바퀴 둘레(m) → 속도(m/s) 계산
  measuredAbsoluteKph = frontEncoderMeasurement.speedMps * 3.6f;
  if (measuredAbsoluteKph < 0.0f) measuredAbsoluteKph = -measuredAbsoluteKph;
  frontEncoderMeasurement.calibrated =
      BroonT870Controller::countsPerWheelRev > 0.0f &&        // 바퀴당 카운트가 설정됐고
      BroonT870Controller::wheelCircumferenceM > 0.0f;        // 바퀴 둘레가 설정된 경우에만 보정됨 표시
  latestEncoderSampleTimeMs = sampleTimeMs;
  lastEncoderSampleMs = nowMs; // 다음 주기 계산을 위해 현재 시각 저장
  return true;
}

void resetRosSpeedControl() {
  BroonT870::resetSpeedPi(rosSpeedPiState);
  BroonT870::resetDriveFeedbackWatchdog(driveFeedbackWatchdogState);
  rosSpeedControlPwm = 0;
}

void updateRosSpeedControl(uint32_t nowMs, bool encoderMeasurementUpdated) {
  const bool driveOutputAuthorized =
      BROON_ENABLE_ACTUATOR_OUTPUTS != 0 && outputsAllowed &&
      !safetyResult.immediateStop;
  if (encoderMeasurementUpdated && selectedMode == MODE_ROS &&
      driveOutputAuthorized) {
    BroonT870::DriveFeedbackWatchdogState overspeedOnlyState = {0UL, false};
    const BroonT870::DriveFeedbackWatchdogResult overspeedCheck =
        BroonT870::updateDriveFeedbackWatchdog(
            overspeedOnlyState, 0.0f, measuredAbsoluteKph,
            frontEncoderMeasurement.deltaCount, 0,
            BroonT870Controller::kDriveFeedbackMinimumPwm,
            BroonT870Controller::kDriveNoFeedbackTimeoutMs,
            BroonT870Controller::kDriveOverspeedLimitKph, nowMs);
    if (overspeedCheck.overspeedFault) {
      driveOverspeedFault = true;
      latchControllerFault(FAULT_DRIVE_OVERSPEED, nowMs);
      return;
    }
  }
  const bool commandCanDrive =
      selectedMode == MODE_ROS && activeCommand.valid && !rcStopActive &&
      !activeCommand.brakeRequested && activeCommand.targetSpeedKph > 0.0f;
  if (!commandCanDrive) {
    resetRosSpeedControl();
    return;
  }
  if (!encoderMeasurementUpdated) return;

  const BroonT870::SpeedPiResult piResult = BroonT870::updateSpeedPi(
      rosSpeedPiState, activeCommand.targetSpeedKph, measuredAbsoluteKph,
      BroonT870Controller::kRosSpeedKp, BroonT870Controller::kRosSpeedKi,
      BroonT870Controller::kRosSpeedDeadbandKph,
      BroonT870Controller::kRosTargetRampKphPerSecond,
      BroonT870Controller::kDriveForwardMaxPwm, latestEncoderSampleTimeMs);
  rosSpeedControlPwm = piResult.pwm;

  if (!driveOutputAuthorized) {
    BroonT870::resetDriveFeedbackWatchdog(driveFeedbackWatchdogState);
    return;
  }
  const BroonT870::DriveFeedbackWatchdogResult watchdog =
      BroonT870::updateDriveFeedbackWatchdog(
          driveFeedbackWatchdogState, activeCommand.targetSpeedKph,
          measuredAbsoluteKph, frontEncoderMeasurement.deltaCount,
          rosSpeedControlPwm, BroonT870Controller::kDriveFeedbackMinimumPwm,
          BroonT870Controller::kDriveNoFeedbackTimeoutMs,
          BroonT870Controller::kDriveOverspeedLimitKph, nowMs);
  if (watchdog.noFeedbackFault) {
    driveNoFeedbackFault = true;
    latchControllerFault(FAULT_DRIVE_NO_FEEDBACK, nowMs);
  } else if (watchdog.overspeedFault) {
    driveOverspeedFault = true;
    latchControllerFault(FAULT_DRIVE_OVERSPEED, nowMs);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 함수: 일정 주기마다 현재 차량 상태를 ROS 또는 시리얼 모니터에 출력
// ─────────────────────────────────────────────────────────────────────────────
void publishOrPrintStatus(uint32_t nowMs) {
#if BROON_ENABLE_ROS || BROON_ENABLE_HUMAN_SERIAL // ROS 또는 사람이 읽는 시리얼 중 하나라도 켜져 있으면
  if (!statusClockStarted) {
    lastStatusMs = nowMs;    // 첫 실행 시 기준 시각 설정
    statusClockStarted = true;
    return;
  }
  if (!BroonT870::hasElapsed(nowMs, lastStatusMs,
                              BroonT870Controller::kStatusIntervalMs)) {
    return; // 출력 주기가 아직 안 됐으면 패스
  }
  lastStatusMs = nowMs; // 마지막 출력 시각 업데이트
#if BROON_ENABLE_ROS // ROS 모드면 컴퓨터로 피드백 전송
  const bool stopRequested = activeCommand.brakeRequested || // 브레이크 요청이거나
                             safetyResult.immediateStop ||   // 즉시 정지 명령이거나
                             !outputsAllowed;                 // 출력 금지이면 → 정지 상태
  rosBridge.publishFeedback(frontEncoderMeasurement, latestSteeringAdc,
                            rcStopActive, stopRequested);    // 엔코더, 조향ADC, RC 정지 상태를 ROS로 전송
#else // 일반(사람용) 시리얼 모드
  Serial.print(F("state="));
  Serial.print(static_cast<uint8_t>(safetyResult.state));   // 현재 안전 상태 숫자 출력
  Serial.print(F(" fault="));
  Serial.print(static_cast<uint8_t>(safetyResult.activeFault)); // 현재 활성 오류 코드 숫자 출력
  Serial.print(F(" mode="));
  Serial.print(selectedMode == MODE_ROS ? F("ROS") : F("RC"));
  Serial.print(F(" stop="));
  Serial.print(rcStopActive ? 1 : 0);
  Serial.print(F(" rc_s="));
  Serial.print(latestRcSnapshot.steerPulseUs);
  Serial.print(latestRcSnapshot.steerValid ? F("V") : F("X"));
  Serial.print(F(" rc_t="));
  Serial.print(latestRcSnapshot.throttlePulseUs);
  Serial.print(latestRcSnapshot.throttleValid ? F("V") : F("X"));
  Serial.print(F(" drive="));
  Serial.print(latestDriveRequestedPwm);
  Serial.print('/');
  Serial.print(latestFrontDrivePwm);
  Serial.print('/');
  Serial.print(latestRearDrivePwm);
  Serial.print(F(" steer="));
  Serial.print(activeCommand.steerTargetAdc);
  Serial.print('/');
  Serial.print(latestSteeringAdc);
  Serial.print('/');
  Serial.print(latestSteeringAuthorizedPwm);
  Serial.print(F(" kph="));
  Serial.print(measuredAbsoluteKph, 2);
  Serial.print(F(" read_us="));
  Serial.print(latestRcReadUs);
  Serial.println();
#endif
#else
  (void)nowMs; // ROS도 시리얼도 꺼져있으면 아무것도 안 함 (nowMs 미사용 경고 억제)
#endif
}

} // namespace (이 중괄호 안의 모든 변수/함수는 이 파일 안에서만 유효)

// ─────────────────────────────────────────────────────────────────────────────
// setup(): 아두이노 전원이 켜졌을 때 딱 한 번만 실행되는 초기화 함수
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  configureMotorPins();                       // 모든 모터 핀을 출력 모드로 설정하고 초기화
  configurationValid = validateConfiguration(); // 모든 설정값 유효성 검사 후 결과 저장

  pinMode(BroonT870Controller::kFrontEncoderAPin, INPUT_PULLUP); // 엔코더 A핀을 입력 모드로 (내부 풀업 저항 사용)
  pinMode(BroonT870Controller::kFrontEncoderBPin, INPUT_PULLUP); // 엔코더 B핀을 입력 모드로 (내부 풀업 저항 사용)
  previousFrontEncoderState = readFrontEncoderState();           // 초기 엔코더 상태 읽어 저장
  attachInterrupt(digitalPinToInterrupt(BroonT870Controller::kFrontEncoderAPin),
                  frontEncoderAIsr, CHANGE); // A핀 신호 변화(CHANGE) 시 frontEncoderAIsr 자동 호출 등록
  attachInterrupt(digitalPinToInterrupt(BroonT870Controller::kFrontEncoderBPin),
                  frontEncoderBIsr, CHANGE); // B핀 신호 변화(CHANGE) 시 frontEncoderBIsr 자동 호출 등록

  pinMode(BroonT870Controller::kStatusLedPin, OUTPUT);          // 상태 표시 LED 핀을 출력 모드로
  digitalWrite(BroonT870Controller::kStatusLedPin, LOW);        // 상태 LED 꺼짐으로 시작
  rcInput.begin(BroonT870Controller::kRcSteerPin,
                BroonT870Controller::kRcThrottlePin,
                BroonT870Controller::kRcAuxPin);                // Uno A0/A1/A2 RC 신호 수신 시작
  BroonT870::resetDrivePair(drivePairState, millis());          // 구동 내부 상태 초기화 (방향 인터락 타이머 등)
  safetyState  = {STATE_BOOT_LOCKED, FAULT_NONE, 0UL, false};   // 안전 상태를 "부팅 잠금"으로 초기화
  safetyResult = {STATE_BOOT_LOCKED, FAULT_NONE, false, true};  // 안전 결과도 "부팅 잠금" 상태로 초기화
  outputsAllowed = false;                                        // 모터 출력 금지로 시작 (잠금 해제 전까지)

#if BROON_ENABLE_HUMAN_SERIAL          // 사람용 시리얼 출력이 켜져 있으면
  Serial.begin(115200);                // 시리얼 통신 시작 (속도: 115200 bps)
#endif
#if BROON_ENABLE_ROS                   // ROS 기능이 켜져 있으면
  rosBridge.begin();                   // ROS 통신 채널 초기화
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// loop(): setup() 이후 아두이노가 꺼질 때까지 무한 반복 실행되는 메인 루프
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
#if BROON_ENABLE_ROS
  rosBridge.spinOnce();                      // ROS에서 들어온 메시지 한 번 처리 (있으면)
#endif
  const uint32_t nowMs = millis();           // 현재 시각 읽기 (아두이노 부팅 후 경과 밀리초)
  const uint32_t nowUs = micros();           // 현재 시각 읽기 (아두이노 부팅 후 경과 마이크로초)
  updateRosCommand(nowMs);                   // ① ROS 컴퓨터 명령 최신값으로 업데이트
  updateInputCommands(nowMs, nowUs);         // ② RC/ROS 입력을 파싱해 실행할 명령 결정
  const bool encoderUpdated = updateFrontEncoderMeasurement(nowMs); // ③ 속도 측정
  updateRosSpeedControl(nowMs, encoderUpdated); // ④ ROS 목표속도 PI 제어
  updateSafety(nowMs);                       // ⑤ 안전 상태 판단 및 모터 출력 허용 여부 결정
  updateSteering(nowMs);                     // ⑥ 조향 모터 P제어 및 안전 가드 적용
  updateDrive(nowMs);                        // ⑦ 앞/뒤 구동 모터 출력 계산 및 적용
  publishOrPrintStatus(nowMs);               // ⑧ 현재 상태를 ROS 또는 시리얼 모니터로 출력
}
