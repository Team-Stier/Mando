# BROON T870 Uno 하위 제어기 운용 안내

이 디렉터리는 임시 시험 스케치가 아니라 차량에 탑재되는 **최종 하위 제어기**다.
상위 계획기와 실제 차량 하드웨어 사이에서 마지막으로 출력을 제한하므로, 소스의
잠금·구성·안전 상태를 우회하지 않는다.

## 역할과 경계

상위 Behavior Planner/ROS는 `/erp42_serial/drive`로 최종 속도·조향·브레이크
요청을 제공한다. Uno는 계획·경로 생성·자율 판단을 하지 않으며, 다음을 담당한다.

- RC 세 채널 수집과 AUX 기반 RC/ROS 모드 선택
- 명령 유효성, RC 원격정지, 중립 대기, 조향 센서·진행 감시, 전후진 인터록을 통한
  `DISARMED`/출력 허용 결정
- 전륜/후륜 구동과 조향의 PWM·DIR 출력, 전륜 D2/D3 사분위 엔코더 측정
- ROS 피드백(`/erp42_serial/feedback`) 발행. 후륜 엔코더와 후륜 속도 피드백은
  존재하지 않는다.

ROS `SerialFeedBack`의 `MorA`는 현재 제어 모드를 `0=RC`, `1=ROS`로 표시한다.
`EStop`은 A1 throttle-cut 또는 RC 스로틀 신호 손실로
발생한 **RC 원격정지 상태**(활성 1), `brake`는 물리 브레이크 위치가 아니라 Uno가 현재 출력 정지를
요구하는 **논리 정지 요청 상태**(명령 brake, 즉시정지 또는 출력 비허용이면 1)다.
`alive`는 피드백을 발행할 때마다 1씩 증가하고 `uint8_t`의 255 다음에는 0으로
랩어라운드한다. `speed`, `steer`, `encoder`의 기존 의미는 유지한다.

RC 모드에서는 유효한 조향·스로틀·AUX RC 채널을 사용한다. ROS 모드도 유효한
AUX 채널이 필요하며, ROS 명령만 계속 들어와도 수신기/AUX가 사라지면 명령은
무효화되어 `DISARMED`가 되어야 한다. 현재는 참고 코드와 같은 `pulseIn()` 방식으로
세 채널을 순차 측정하며 채널당 timeout은 50 ms다. 세 신호가 모두 없으면 한 루프가
최대 약 150 ms 막힐 수 있으므로 실제 벤치에서 신호 손실 정지 시간을 다시 확인한다.
조향·스로틀·AUX의 최소/중앙/최대값은 각각
`kRcSteerCalibration`, `kRcThrottleCalibration`, `kRcAuxCalibration`에 따로
전사한다. 조향값은 조향 매핑에만, 스로틀값은 스로틀 매핑에만 사용하며 AUX
범위와 두 임계값은 모드 선택 유효성에만 사용한다.

`DriveCmd.KPH`는 부호 없는 `uint16_t`이므로 현재 ROS 경로는 전진 전용이다.
큰 unsigned 값을 음수 후진으로 해석하지 않는다. 상위 시스템이 기어/후진 필드와
안전 정책을 정의·검증하기 전에는 ROS 후진을 허용하지 않는다.

## 현재 Uno 핀맵

다음 표는 `ControllerConfig.h`에 반영된 **Arduino Uno 측 핀 배치**다.

| Uno 핀 | 기능 | 설정 항목 | 주의사항 |
|---|---|---|---|
| D0 (RX) | USB Serial 수신 | Arduino/rosserial 기본 직렬 | ROS 사용 중 외부 장치를 연결하지 않는다. |
| D1 (TX) | USB Serial 송신 | Arduino/rosserial 기본 직렬 | ROS 데이터에 `Serial.print`를 섞지 않는다. |
| D2 | 전륜 엔코더 A(임시) | `kFrontEncoderAPin` | A/B 반대이면 카운트 부호가 반전됨 |
| D3 | 전륜 엔코더 B | `kFrontEncoderBPin` | 외부 인터럽트 입력, 후륜 엔코더는 없음 |
| D4 | 조향 DIR | `kSteeringPins.dir` | 디지털 방향 출력 |
| D5 | 조향 PWM | `kSteeringPins.pwm` | Uno 하드웨어 PWM 출력 |
| D6 | 후륜 구동 PWM | `kRearDrivePins.pwm` | Uno 하드웨어 PWM 출력 |
| D7 | 후륜 구동 DIR | `kRearDrivePins.dir` | D7은 PWM 핀이 아니므로 DIR로만 사용 |
| D8 | 전륜 구동 DIR | `kFrontDrivePins.dir` | D8은 PWM 핀이 아님 |
| D9 | 전륜 구동 PWM | `kFrontDrivePins.pwm` | Uno 하드웨어 PWM 출력 |
| D13 | 상태 LED | `kStatusLedPin` | Uno 내장 LED |
| A0 | RC 조향 | `kRcSteerPin` | 수신기와 Uno GND 공통 |
| A1 | RC 스로틀 | `kRcThrottlePin` | 수신기와 Uno GND 공통 |
| A2 | RC AUX | `kRcAuxPin` | RC/ROS 모드 선택 채널 |
| A4 | 조향 포텐시오미터 | `kSteeringSensorPin` | 0~5 V 범위의 아날로그 입력만 허용 |

전륜·후륜·조향 PWM은 D9·D6·D5의 Uno 하드웨어 PWM 기능을 사용한다.
후륜은 PWM D6, DIR D7이며 D7에는 `analogWrite()`를 사용하지 않는다.
D2·D3은 전륜 사분위 엔코더 외부 인터럽트용이며, A0·A1·A2 RC 입력은 이전 참고
코드와 같은 `pulseIn(..., HIGH, 50000)` 방식으로 순차 캡처한다.

## 벤치 스케치 업로드와 실행 방법

벤치 스케치는 생산용 제어기의 RC/ROS 모드가 아니라, 특정 하드웨어만 확인하기 위해 Uno에 임시로 올리는 독립 시험 프로그램이다. Uno에는 한 번에 스케치 하나만 저장되므로 새 스케치를 업로드하면 이전 스케치는 덮어써진다. 업로드가 끝나면 Uno가 자동으로 리셋되어 즉시 실행되며 별도의 실행 버튼은 없다. 모든 벤치 시험이 끝난 뒤 생산용 `BROON_T870_Uno_Controller.ino`를 다시 업로드한다.

Arduino IDE에서는 다음 순서로 진행한다.

1. 처음에는 차량의 24 V 모터 전원을 차단하고 Uno를 USB로만 PC에 연결한다.
2. 시험할 `.ino` 파일을 연다. Arduino IDE가 같은 이름의 폴더 사용을 요구하면 해당 스케치 폴더 전체를 그대로 연다.
3. `도구 > 보드`에서 `Arduino Uno`, `도구 > 포트`에서 Uno의 `COM` 포트를 선택한다.
4. 먼저 `검증(✓)`으로 컴파일한 뒤 `업로드(→)`를 누른다.
5. `업로드 완료`가 표시되면 시리얼 모니터를 열고 baud rate를 `115200`으로 설정한다.
6. 다른 시험으로 넘어갈 때는 다음 벤치 스케치를 업로드한다. 이전 스케치를 별도로 삭제할 필요는 없다.

### 벤치 스케치별 사용법

| 스케치 | 차량 없이 가능한 범위 | 필요한 장비 | 실행 및 확인 방법 |
|---|---|---|---|
| `tests/BroonT870CoreSelfTest/BroonT870CoreSelfTest.ino` | 전부 가능 | Uno와 USB 케이블 | 업로드 직후 한 번 실행된다. 시리얼 모니터에서 `PASS`/`FAIL`과 마지막 `TOTAL pass=... fail=0`을 확인한다. 다시 실행하려면 Uno의 RESET 버튼을 누른다. |
| `tests/BroonT870EncoderBench/BroonT870EncoderBench.ino` | 전체 차량은 불필요하지만 전륜 엔코더 또는 동일한 시험 장치가 필요 | Uno, 전륜 A/B 엔코더, 전압이 확인된 신호 배선 | D2/D3의 전륜 카운트를 100 ms마다 출력한다. 시리얼 모니터에서 `ZERO`를 전송해 0으로 만든 뒤 정방향과 역방향으로 각각 10회전시킨다. 양쪽 부호가 반대인지와 CPR 절댓값이 비슷한지 기록한다. |
| `tests/BroonT870RcBench/BroonT870RcBench.ino` | 차량 없이 가능 | Uno, RC 송신기와 수신기 | 업로드하면 조향·스로틀·AUX의 펄스폭, 유효 여부, AUX 영역을 100 ms마다 출력한다. 수신기 전원을 제거했을 때 세 채널이 설정된 timeout 안에 모두 invalid가 되는지 확인한다. |
| `tests/BroonT870OutputBench/BroonT870OutputBench.ino` | 전체 차량은 불필요하지만 실제 출력 시험에는 드라이버와 모터가 필요 | Uno, 시험할 모터 드라이버와 모터, 퓨즈, 물리 24 V 차단 장치 | 기본 매크로가 `BROON_OUTPUT_BENCH_ENABLE_OUTPUTS=0`이므로 업로드만으로는 모터가 움직이지 않는다. 격리된 단일 채널 시험을 준비한 경우에만 이를 `1`로 바꾸고 다시 업로드한다. 시리얼 모니터에서 `ARM` 후 `FRONT 20`, `REAR -20`, `STEER 15`처럼 한 채널만 명령한다. `STOP`으로 즉시 멈출 수 있고, PWM 절댓값은 30으로 제한되며 300 ms 후 자동 정지한다. |

### 약 75 PPR 측정값과 실제 설정값

분해 시험에서 확인한 약 75 PPR은 엔코더가 동작한다는 기준값으로 사용하되,
`ControllerConfig.h`의 `countsPerWheelRev`에 곧바로 `75`를 입력하지 않는다.
PPR 표기 방식은 측정 방법에 따라 다르고, 현재 코드는 전륜 엔코더 A/B 채널의
상승·하강 변화를 모두 세는 사분위 4체배 방식이다. 75 PPR이 한 채널의 펄스 수라면
Uno에서 약 300 count/rev로 측정될 수 있다. 반대로 75가 이미 같은 코드의 4체배
결과라면 다시 4를 곱하지 않는다.

모터 24 V 전원을 차단하고 바퀴를 띄운 뒤 `BroonT870EncoderBench`를 업로드한다.
시리얼 모니터에서 `ZERO`를 입력하고 **실제 전륜 바퀴를** 정방향 10회전,
역방향 10회전시킨다. 엔코더가 모터축에 있으면 감속비까지 포함되어야 하므로
엔코더축이 아니라 바퀴 회전수를 기준으로 한다. 최종값은 다음처럼 계산한다.

`countsPerWheelRev = (abs(정방향 10회전 카운트) + abs(역방향 10회전 카운트)) / 20`

예를 들어 정방향 `+2980`, 역방향 `-2990`이면 설정 후보는
`(2980 + 2990) / 20 = 298.5 count/rev`이다. 두 방향의 부호가 반대이고 각각의
절댓값을 10으로 나눈 결과가 비슷한지 확인한 뒤, 계산값을
`CALIBRATION_RECORD_KO.md`에 기록하고 `ControllerConfig.h`로 전사한다.

`BroonT870OutputBench`는 안전상 한 번에 모터 하나만 시험한다. 현재 패키지에는 RC/ROS 없이 전륜과 후륜을 동시에 움직이는 전용 페어 벤치 스케치가 없다. 전·후륜 페어 시험은 모든 센서·RC 원격정지·방향 보정이 끝난 뒤 생산용 제어기의 RC 모드로 진행하거나, PWM 30/300 ms 제한과 조향 출력 0을 강제하는 별도 페어 벤치 스케치를 추가한 뒤 진행한다. 검증 전에는 ROS 모드로 페어 시험을 시작하지 않는다.

### 시험 단계별 전원 원칙

- Core self-test는 USB 전원만 사용한다.
- Encoder/RC bench는 모터 24 V 전원을 차단한 상태에서 센서 신호만 연결한다.
- Output bench부터는 바퀴 또는 모터를 지면에서 분리하고, 퓨즈와 즉시 접근 가능한 물리 24 V 차단 장치를 사용한다.
- Uno 입출력에는 24 V를 직접 연결하지 않는다. 엔코더와 수신기 신호 전압이 Uno 입력 허용 범위인지 먼저 측정한다.
- 모터 전원을 연결할 때는 Uno와 드라이버의 신호 GND를 공통으로 구성하되, 전력 배선과 신호 배선의 극성·정격을 각각 확인한다.
- 벤치 결과는 `CALIBRATION_RECORD_KO.md`의 해당 항목에 기록한다.

## 교정값 측정 및 ControllerConfig.h 입력

교정은 코드를 차량에 맞게 자동 조정하는 기능이 아니라, 벤치에서 측정한 값을
`CALIBRATION_RECORD_KO.md`에 기록한 뒤 `ControllerConfig.h`로 수동 전사하는
과정이다. 아래 표의 단위와 목적지를 지킨다. 이 문서의 설명용 예시 숫자를 실제
설정값으로 사용하지 않는다.

| 교정 항목 | 측정 방법 | 단위 | `ControllerConfig.h` 목적지 |
|---|---|---|---|
| 전륜 1회전 카운트 | EncoderBench에서 정·역방향 실제 바퀴 각 10회전 | count/rev | `countsPerWheelRev` |
| 하중 상태 타이어 둘레 | 탑재 상태에서 바퀴 1회전 실제 이동거리 측정 | m | `wheelCircumferenceM` |
| 조향 좌 기계 끝 | 출력 차단 상태에서 안전한 방법으로 이동 가능한 왼쪽 한계 확인 | ADC | `kSteeringCalibration.leftMechanicalAdc` |
| 조향 좌 안전 끝 | 기계 끝보다 안쪽에 출력 제한점을 지정 | ADC | `kSteeringCalibration.leftSafeAdc` |
| 조향 중앙 | 바퀴가 직진하는 위치의 포텐시오미터값 | ADC | `kSteeringCalibration.centerAdc` |
| 조향 우 안전 끝 | 기계 끝보다 안쪽에 출력 제한점을 지정 | ADC | `kSteeringCalibration.rightSafeAdc` |
| 조향 우 기계 끝 | 출력 차단 상태에서 안전한 방법으로 이동 가능한 오른쪽 한계 확인 | ADC | `kSteeringCalibration.rightMechanicalAdc` |
| 조향 ADC 방향 | 오른쪽으로 이동할 때 ADC가 증가하는지 확인 | true/false | `kSteeringCalibration.adcIncreasesRight` |
| RC 조향 최소·중앙·최대 | RcBench에서 끝점과 중립을 기록 | µs | `kRcSteerCalibration` |
| RC 스로틀 최소·중앙·최대 | RcBench에서 전후 끝점과 중립을 기록 | µs | `kRcThrottleCalibration` |
| RC AUX 최소·중앙·최대 | RcBench에서 스위치 각 위치를 기록 | µs | `kRcAuxCalibration` |
| RC throttle-cut 펄스 | throttle-cut 활성·해제 상태를 각각 측정 | µs | `kRcStopThresholdUs` |
| 전륜·후륜 안전 PWM | OutputBench에서 한 채널씩 낮은 값부터 확인 | 0~255 | `kDriveForwardMaxPwm`, `kDriveReverseMaxPwm` |
| 조향 최소·최대 PWM | 기계 끝에서 떨어진 구간에서 낮은 값부터 확인 | 0~255 | `kSteeringMinimumPwm`, `kSteeringMaximumPwm` |
| 모터 실제 방향 | 각 모터의 양·음 명령과 실제 움직임 비교 | true/false | 세 `...DirectionInverted` 값 |

조향 5점 ADC는 코드가 요구하는 물리적 순서로 입력해야 한다. ADC가 오른쪽으로
증가하면 일반적으로 `leftMechanical < leftSafe < center < rightSafe <
rightMechanical` 순서이고, 감소하면 반대 순서다. 안전 끝은 기계 끝과 같게 두지
않는다. 조향기어를 기계 끝에 계속 밀어붙여 값을 측정하지 않는다.

조향 제어에는 다음 항목도 실기에서 낮은 값부터 조정한다.

- `kSteeringKp`: 목표 ADC 오차에 곱하는 비례 이득
- `kSteeringDeadbandAdc`: 중앙 부근에서 조향 출력을 0으로 보는 허용 오차
- `kSteeringApproachBandAdc`: 안전 끝에 접근할 때 PWM을 줄이는 구간
- `kSteeringRecoveryPwm`: 안전 범위 밖에서 안쪽으로 복귀할 때 허용하는 PWM
- `kSteeringProgressDeltaAdc`: 조향이 실제 움직였다고 판정하는 최소 ADC 변화
- `kSteeringNoProgressMs`: ADC 무진행 스톨 제한시간
- `kSteeringMaxDriveMs`: 연속구동 제한시간이며 `0`이면 fault 4 감시 비활성

전·후륜 구동은 `kDriveAccelerationRampStep=10`으로 가속하고,
`kDriveDecelerationRampStep=20`으로 더 빠르게 감속한다. RC 스로틀 중립에서는
램프를 기다리지 않고 PWM을 즉시 0으로 만든다. 정·역방향 전환은 기존처럼
`kDirectionInterlockMs` 동안 PWM 0을 유지한 뒤 방향을 바꾼다.

측정값 전사와 잠금 해제는 다음 순서를 지킨다.

1. `BROON_ENABLE_ACTUATOR_OUTPUTS=0`과 24 V 차단 상태에서 센서값을 측정한다.
2. 모든 결과를 `CALIBRATION_RECORD_KO.md`에 기록한다.
3. 기록값을 `ControllerConfig.h`의 대응 항목에 입력한다.
4. 출력 잠금 `0` 상태로 컴파일하여 설정 검사가 통과하는지 확인한다.
5. 조향 5점·방향·안전 PWM을 재검토한 뒤
   `BROON_STEERING_CALIBRATION_CONFIRMED=1`로 바꾼다.
6. RC throttle-cut 활성값이 `kRcStopThresholdUs`보다 낮고, 신호 손실도 정지로
   처리되는지 독립 확인한다.
7. 바퀴를 띄우고 물리 24 V 차단장치와 퓨즈를 준비한 뒤 각 채널을 다시 확인한다.
8. 모든 서명과 승인이 끝난 마지막 단계에서만
   `BROON_ENABLE_ACTUATOR_OUTPUTS=1`로 바꾼다.

두 잠금을 모두 `1`로 바꿔도 count/rev, 타이어 둘레, 조향 5점, 조향 이득과
최대 PWM 또는 구동 최대 PWM이 유효하지 않으면 생산 스케치는
출력을 허용하지 않는다.

## 빌드와 통신 조건

Arduino IDE 또는 CLI에서 보드는 반드시 **Arduino Uno (`arduino:avr:uno`)**로
선택한다. ROS를 끈 기본 빌드는 로컬 `libraries`만 사용한다.

ROS를 켜는 빌드는 이 PC에서 확인된 실제 생성 라이브러리 부모 경로
`C:\Users\suhyeon\Documents\Arduino\libraries`와 그 안의
`ros_lib`, 생성된 `erp42_msgs` 메시지가 모두 필요하다. 이 경로는 컴퓨터마다
다르며 저장소에 가짜 `ros_lib` 헤더를 추가하지 않는다.

Uno ROS 빌드는 현재 정적 컴파일 기준 플래시 18,484바이트, 전역 SRAM 1,475바이트를
사용해 SRAM 573바이트가 남는다. 컴파일 성공만으로 런타임 스택 여유가 검증되는
것은 아니므로 지속 rosserial 통신과 순차 RC `pulseIn()` 측정이 동시에 동작하는 벤치에서
리셋·패킷 손실 여부를 확인한다.

ROS를 켠 USB 직렬 포트에는 사람이 읽는 `Serial.print` 텍스트를 섞지 않는다.
`BROON_ENABLE_ROS=1`이면 `BROON_ENABLE_HUMAN_SERIAL=0`이어야 하며,
rosserial 트래픽 중 사람이 읽는 텍스트가 전혀 없는지를 실기에서 점검한다.

## 출력 잠금과 교정

`BuildOptions.h`의 다음 두 잠금은 현재 모두 `0`이다.

- `BROON_ENABLE_ACTUATOR_OUTPUTS=0`: 모터 PWM 출력을 차단한다.
- `BROON_STEERING_CALIBRATION_CONFIRMED=0`: 교정 미확인 상태를 유지한다.

물리 E-stop 입력은 사용하지 않는다. A1 throttle-cut 펄스가
`kRcStopThresholdUs`보다 낮거나 RC 스로틀 신호가 끊기면 RC 원격정지를 활성화한다.
이 소프트웨어 정지는 독립적인 물리 전원 차단장치를 대신하지 않는다.

`ControllerConfig.h`에는 실제 전륜 1회전에서 확인한 `300 count/rev`와 지름
270 mm로 계산한 임시 원주 `0.84823 m`가 들어 있다. 원주는 하중 상태에서 바퀴를
한 바퀴 굴린 실제 이동거리로 최종 보정해야 한다. 조향은 좌 기계·명령 끝 1020,
직진 600, 우 명령·기계 끝 180으로 입력했다. 별도 소프트웨어 끝점 여유가 없으므로 직진값과 PWM은
리프트 휠 실기 검증 전의 보수적 초깃값이다. [교정 기록](CALIBRATION_RECORD_KO.md)을
완성하고 독립 검토·리프트 휠 시험·명시적 승인까지 끝내기 전에는 잠금을 변경하지
않는다.

## 정규화 조향 제어와 ROS 속도 PI 제어

조향은 포텐시오미터 범위를 `좌 1020 = +1000`, `직진 600 = 0`,
`우 180 = -1000`으로 나눠 정규화한다. 목표와 현재 위치 차이가 약 21 ADC면
좌우 모두 PWM 155, 약 42 ADC 이상이면 좌우 모두 PWM 210을 요청한다.
현재 설정은 데드밴드 8 ADC, 일반 최소 PWM 100, 최대 PWM 210이다. 안전 끝에서
100 ADC 이내로 접근하면 PWM을 선형으로 줄이되, 허용된 비영 출력은 정지마찰을
이길 수 있도록 PWM 60 아래로 낮추지 않는다. 안전범위 밖에서는 안쪽 복귀
방향으로만 최대 PWM 60을 허용한다. RC 조향은 `1100 → 1020`, `1450 → 600`,
`1800 → 180`으로 중앙을 기준으로 나눠 보간하므로 실제 직진값을
보존한다. D항은 ADC 노이즈를 키울 수 있고 I항은 기계 끝에서 누적될 수 있어
초기 조향 제어에는 사용하지 않는다.

목표와 현재 위치 차이가 5 ADC 이내가 되면 조향 출력을 0으로 정착시키고,
정착 후에는 오차가 18 ADC 이상 벌어져야 다시 움직인다. 이동 중 요청 방향이
반대로 바뀌면 한 번의 제어 갱신 동안 PWM 0을 출력한 뒤 반대 방향을 허용한다.
따라서 목표에서 멀 때의 PWM 100~210 반응을 높이면서 목표 근처의 반복 방향
반전을 억제한다.

좌·우 끝점 5 ADC 이내를 목표로 할 때는 끝점 전용 히스테리시스를 사용한다.
목표 오차가 25 ADC 이내면 출력을 0으로 유지하고, ADC 피드백이 튀어도 오차가
60 ADC 이상 벌어지기 전에는 다시 구동하지 않는다. 이는 최대 조향에서 관찰된
`-60/0/+60` 반복 출력을 억제하기 위한 설정이다.

실제 조향 출력이 허가된 상태에서 1초 동안 포텐시오미터가 2 ADC 이상 움직이지
않으면 fault 3(`FAULT_STEERING_STALL`)을 래치한다. 현재 `kSteeringMaxDriveMs=0`이므로
ADC가 정상적으로 움직이는 동안에는 조향 지속시간만으로 fault 4가 발생하지 않는다.
상태 로그의 `steer=target/adc/pwm`은 목표 ADC, 현재 ADC, 실제 허가 PWM을 표시한다.
fault는 자동 해제되지 않으므로 원인을 바로잡은
뒤 Arduino를 재부팅한다.

사람용 상태 로그의 `read_us`는 A0/A1/A2 세 채널을 `pulseIn()`으로 읽는 데 걸린
총 시간이다. 이 값이 크면 조종기 입력이 반영되기 시작하는 지연이 커질 수 있지만,
PWM이 인가된 뒤 조향 모터 자체의 이동 속도는 `steer`의 마지막 PWM과 ADC 변화로 판단한다.
PWM이 `-210` 또는 `+210`인데도 ADC가 천천히 변하면 남은 병목은 RC 매핑이
아니라 PWM 상한, 전원·드라이버 또는 조향 기구 쪽이다.

RC 사람용 로그는 `state fault mode stop rc_s rc_t drive=t/f/r steer=t/a/p kph read_us`
순서만 출력한다. `drive`는 요청/전륜/후륜 PWM이고 `steer`는 목표/현재/PWM이다.
ROS 전용 목표속도와 PI 출력은 이 로그에서 제외한다.

송신기를 꺼도 수신기가 저장된 페일세이프 PWM을 계속 출력할 수 있다. 이때 `rc_s`가
약 1335~1340 µs로 유효하게 남으면 현재 매핑에서 `steer` 목표가 약 730으로 표시된다.
유효 PWM만으로는 송신기 연결 여부를 구분할 수 없으므로, 실제 정지는 수신기의
스로틀 페일세이프가 `stop=1`이 되는 값으로 저장됐는지 확인해야 한다.

ROS 모드의 `DriveCmd.KPH`는 PWM 비율이 아니라 **목표속도**다. 전륜 D2/D3
엔코더로 100 ms마다 측정한 속도와 목표속도의 차이를 PI 제어해 전·후륜 공통
PWM을 만든다. 현재 제한은 목표 3 km/h, 양의 PI 출력 최소 PWM 80, 최대 PWM 200,
목표 램프 1 km/h/s이며, Kp 20, Ki 8, 속도 데드밴드 0.12 km/h다. 계산 출력이
0이면 PWM도 0이고, 계산 출력이 1~79이면 기동을 위해 80으로 올린다. 적분 포화
방지를 적용하고 목표 0,
brake, ROS timeout, RC 원격정지, RC 모드 전환 시 PI 상태를 초기화한다. RC
모드는 엔코더 폐루프가 아니라 기존 스로틀-to-PWM 방식이다.

현재 토픽/벤치 시험 설정에서는 전륜 엔코더 경로가 아직 검증되지 않아 무응답
timeout을 0으로 두고 fault 5(`FAULT_DRIVE_NO_FEEDBACK`)를 비활성화했다. 측정속도가
4 km/h를 넘는 fault 6(`FAULT_DRIVE_OVERSPEED`) 감시는 유지한다. 전륜 센서 하나만
있으므로 후륜 개별 속도나 전·후륜 속도 차이는 검출할 수 없다. 실차 주행 전에는
엔코더를 검증하고 무응답 timeout을 다시 활성화해야 한다.

## RC 우선 검증과 ROS 활성화

ROS를 연결하기 전에 생산용 하위 제어기를 RC 모드에서 먼저 검증한다. RC 단계와
ROS 단계는 `BuildOptions.h` 설정과 USB 포트 사용 방식이 다르므로 한 번에 섞지
않는다.

### 1단계: RC 전용 센서·통신 확인

처음에는 다음 설정을 유지한다.

```cpp
#define BROON_ENABLE_ACTUATOR_OUTPUTS 0
#define BROON_ENABLE_ROS 0
#define BROON_ENABLE_HUMAN_SERIAL 1
```

생산 스케치를 업로드하고 시리얼 모니터를 `115200` baud로 연다. RC 조향·스로틀·
AUX, RC 원격정지, 조향 ADC, 전륜 엔코더와 `state`가 예상대로 변하는지 확인한다.
수신기 전원을 끄면 세 RC 채널이 `kRcTimeoutMs` 안에 무효가 되고 상태가
`DISARMED`가 되어야 한다.

### 2단계: RC 리프트 휠 출력 확인

교정·서명·RC 원격정지 검증이 완료된 경우에만 앞 절의 순서로 두 잠금을 해제한다.
바퀴를 띄운 상태에서 낮은 명령부터 전륜·후륜·조향 방향, 램프, 조향 안전 끝,
throttle-cut과 수신기 손실 시 PWM 0을 확인한다. RC 검증이 끝나기 전에는 ROS 빌드로
출력 시험을 시작하지 않는다.

### 3단계: ROS 라이브러리 준비

Ubuntu ROS 환경에 `erp42_msgs`, `rosserial_arduino`, `rosserial_python`이 있어야
한다. Ubuntu에서 Arduino용 헤더를 생성하는 예시는 다음과 같다.

```bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries erp42_msgs std_msgs
```

생성된 `ros_lib`에 `erp42_msgs/DriveCmd.h`와
`erp42_msgs/SerialFeedBack.h`가 실제로 존재하는지 확인한다. 다른 PC에서 빌드할
때는 그 PC의 Arduino 라이브러리 경로를 사용하고, 저장소에 임시 가짜 메시지
헤더를 만들지 않는다.

### 4단계: ROS 빌드와 무출력 통신 시험

다음처럼 ROS만 USB 직렬 포트를 사용하게 설정한다.

```cpp
#define BROON_ENABLE_ACTUATOR_OUTPUTS 0
#define BROON_ENABLE_ROS 1
#define BROON_ENABLE_HUMAN_SERIAL 0
```

현재 설치된 `ros_lib`의 기본 baud는 `57600`이다. Arduino IDE 시리얼 모니터를
닫고 Ubuntu에서 다음처럼 rosserial 노드를 실행한다. 포트 이름은 실제 Uno에
맞춘다.

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

다른 터미널에서 피드백을 확인한다.

```bash
rostopic echo /erp42_serial/feedback
```

바퀴를 띄우고 출력 잠금을 유지한 상태에서 낮은 전진·중앙 조향 명령을 보낸다.

```bash
rostopic pub -r 10 /erp42_serial/drive erp42_msgs/DriveCmd \
  "{KPH: 1, Deg: 0, brake: 0}"
```

ROS 모드도 유효한 RC AUX 신호가 필요하다. 현재 설정에서는 AUX가
`kAuxRosThresholdUs` 이하이면 ROS, `kAuxRcThresholdUs` 이상이면 RC를 선택하고
중간 영역에서는 이전 모드를 유지한다. 수신기/AUX를 끄거나 ROS 명령을 중단하면
각 timeout 안에 `DISARMED` 및 PWM 0이 되어야 한다.

### 5단계: ROS 출력 검증

ROS 통신, timeout, 수신기 손실과 피드백이 무출력 상태에서 모두 검증된 다음에만
승인된 출력 잠금을 적용해 가장 낮은 속도부터 시험한다. 현재
`erp42_msgs/DriveCmd.KPH`가 `uint16_t`이므로 ROS 경로는 전진 전용이다. 후진은
상위 기어 정책과 별도 메시지 정의가 생기기 전까지 시험하지 않는다.

출력 잠금 해제 후에는 ROS 모드를 선택한 상태에서 먼저 `KPH: 0`, `brake: 0` 명령을
최소 0.5초 동안 연속 전송해 중립 ARM 절차를 통과시킨다. 그 다음 `KPH: 1`부터
시험한다. 전원이 다시 켜졌거나 timeout·RC 원격정지·brake로 DISARM된 뒤에도 같은
0 km/h 중립 유지 절차를 다시 거친다.

## 물리 안전 전제

- 즉시 접근 가능한 물리 24 V 차단장치, 공통 신호 GND, 적절한 퓨즈를 확인한다.
- 엔코더 전압이 Uno 입력 허용 범위인지 계측한다.
- 모든 출력/방향/교정 시험은 바퀴를 지면에서 띄운 상태에서 한다.
- CAN은 RC/ROS 기반 차량 제어가 검증될 때까지 의도적으로 보류한다. 현재 후륜은
  D6/D7을 사용하므로 D10/D11은 후륜 출력에 사용하지 않는다.

이 항목과 정적 빌드가 통과해도 도로 주행 안전을 증명하지 않는다.

## 권장 시험 순서

1. **센서:** 출력 잠금 `0`에서 조향 ADC, RC throttle-cut, 전륜 엔코더 A/B와
   전·후 10회전 카운트를 점검한다. 후륜 피드백이 없음을 기록한다.
2. **각 모터 단독:** 바퀴를 띄운 채 전륜, 후륜, 조향 각각의 PWM·DIR과 실제
   방향 반전 플래그를 검증한다.
3. **쌍 구동:** 전·후륜이 같은 논리 명령에서 올바른 방향·램프·0 PWM 정지와
   방향 전환 대기를 보이는지 확인한다. 논리 PWM 0과 DISARM 동안 물리 DIR은
   그대로 유지되고, 대기가 끝난 뒤 첫 비영 PWM 직전에만 PWM 0 상태에서 DIR이
   바뀌는지 양방향으로 확인한다.
4. **RC:** 세 RC 채널 범위/AUX 모드를 검증한 뒤 수신기 전원을 끄고 세 채널이
   100 ms 이내 무효인지, 활성 상태가 `DISARMED`인지 점검한다.
5. **RC 저속 지상 시험:** 통제된 폐쇄 구역에서 차량을 무인·무하중으로 두고,
   감시자와 즉시 접근 가능한 물리 24 V 차단장치를 배치한다. 가장 낮은 PWM에서
   직선 단거리 주행을 수행해 전륜 엔코더 속도·거리와 실제 거리를 비교한다.
   후륜이 같은 명령을 따르는지 관찰하고, throttle-cut과 수신기 손실 정지를 각각
   확인한다.
6. **선택적 ROS:** RC 검증이 완료된 뒤 ROS 명령 타임아웃과 수신기 손실 각각에서 `DISARMED` 및 0 출력이
   되는지 확인한다. rosserial 지속 트래픽 중 통신 안정성과 스택 여유를 별도로
   계측하며, 사람 읽기 텍스트가 없는지도 확인한다.
   조향 stall 타이머는 실제로 허가된 비영 조향 출력에만 진행되어야 한다.
   BOOT/DISARM/brake/RC 원격정지/timeout에는 타이머가 정지·초기화되고, 재ARM 후
   새 창으로 시작하는지 확인한다. 현재 연속구동 fault 4 타이머는 비활성이다.
7. **선택적 ROS 지상 시험:** ROS 벤치가 끝난 경우에만 같은 폐쇄 구역에서 가장
   낮은 속도로 진행한다. 좌·우 조향이 기계 끝을 치지 않고 안전 한계에 접근하는지,
   ROS timeout과 RC 수신기 손실에서 정지하는지 확인한다. 매 회차 사이 열·전류·
   기계적 이상을 점검하고, 기록과 서명 없이는 탑승자를 태우거나 속도/PWM을
   높이지 않는다.

ROS timeout·수신기 손실 `DISARMED` 시험과 지속 rosserial 트래픽/스택 여유
검증은 현재 정적 컴파일만으로 완료된 것으로 간주할 수 없는 물리 게이트다.
