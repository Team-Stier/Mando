# T870 CAN 텔레메트리와 CSV 로거

## 범위

이 기능은 차량 상태를 기록하기 위한 **송신 전용 모니터링 경로**다. CAN 프레임은
RC 또는 ROS 명령을 대신하지 않으며 모터 출력, 안전 상태, fault 정책을 변경하지
않는다. MCP2515 초기화 실패, 버스 단선, ACK 부재 또는 송신 버퍼 포화도 차량 제어를
정지시키지 않고 `DroppedFrames`와 MCP2515 오류 카운터로만 관찰한다.

```text
T870 제어 Uno(A) -- MCP2515/TJA1050 == CAN == MCP2515/TJA1050 -- 로거 Uno(B)
                                                                |
                                                              USB
                                                                |
                                                        Ubuntu CSV 저장
```

현재 규격은 다음과 같다.

- CAN 2.0A 표준 11-bit ID
- 500 kbit/s
- MCP2515 모듈 오실레이터 8 MHz(`8.000` 표기 확인)
- 데이터 바이트 순서: little-endian(Intel)
- 배터리 전압: 이번 버전에서 제외
- 명령 수신: 제외. 제어 Uno는 CAN 송신만 수행

## 작업본과 코드 위치

실차 시험을 통과한 최신 RC/ROS 제어 코드와 CAN 통합 작업본은 로컬 Git 브랜치
`codex/can-on-tested-controller`에 있다. GitHub `main`에는 아직 CAN 통합본을
push하지 않았으므로 원격 `main` 파일만으로는 CAN이 송신되지 않는다.

| 용도 | `BROON_T870_Uno_Controller` 폴더 기준 위치 |
|---|---|
| T870 제어 Uno(A) 메인 스케치 | `BROON_T870_Uno_Controller.ino` |
| 핀맵·제어 파라미터 | `ControllerConfig.h` |
| RC/ROS/CAN 빌드 스위치 | `BuildOptions.h` |
| CAN 상태 송신기 | `CanTelemetry.h`, `CanTelemetry.cpp` |
| MCP2515 드라이버 | `T870Mcp2515.h` |
| CAN 프레임 인코딩 규격 | `T870CanProtocol.h` |
| CAN DBC | `can/T870_CAN.dbc` |
| DBC 재해석·교차검증기 | `can/decode_can_frames.py` |
| 로거 Uno(B) 스케치 | `logger/T870CanCsvLogger/T870CanCsvLogger.ino` |
| 캡처·진단·DBC 통합 실행기 | `logger/capture_and_diagnose.py` |
| 단순 CSV 저장기 | `logger/save_serial_csv.py` |
| CAN 프로토콜 자체시험 | `tests/T870CanProtocolSelfTest/T870CanProtocolSelfTest.ino` |

Arduino IDE에서는 `.ino` 한 개만 복사하지 말고
`BROON_T870_Uno_Controller` 폴더 전체를 사용한다. 메인 스케치는 같은 폴더의
헤더와 `.cpp` 파일을 함께 컴파일한다.

## 준비물

- T870 제어용 Arduino Uno(A) 1개
- CAN 로거용 Arduino Uno(B) 1개
- `8.000` 크리스털과 TJA1050이 장착된 MCP2515 모듈 2개
- CAN-H/CAN-L용 트위스트 페어
- 120 ohm 종단저항 2개(모듈 내장 여부를 먼저 확인)
- 멀티미터와 USB 케이블 2개

초기 시험은 차량의 24 V 모터 전원을 차단하고 두 Uno를 USB 또는 검증된 5 V
전원으로만 구동한다. TJA1050 모듈과 Uno에 24 V를 직접 연결하지 않는다.

## 확정 핀 배치

실차 시험 완료 코드의 후륜 D6/D7 배선은 그대로 유지하고 상태 LED만 제거한다.
D10~D13은 Uno 하드웨어 SPI로 예약하며 다음 배치를 사용한다.

| 기능 | 현재 제어 코드 | CAN 통합본 |
|---|---:|---:|
| 후륜 PWM | D6 | D6(유지) |
| 후륜 DIR | D7 | D7(유지) |
| 상태 LED | D13 | 미사용(코드 제거) |
| MCP2515 CS | 없음 | D10 |
| MCP2515 MOSI | 없음 | D11 |
| MCP2515 MISO | 없음 | D12 |
| MCP2515 SCK | 없음 | D13 |

### 제어 Uno(A)와 MCP2515

| 제어 Uno(A) | MCP2515/TJA1050 |
|---|---|
| 5 V | VCC(모듈 정격 확인) |
| GND | GND |
| D10 | CS |
| D11 | MOSI/SI |
| D12 | MISO/SO |
| D13 | SCK |
| 연결하지 않음 | INT |

제어 Uno(A)의 MCP2515 INT는 연결하지 않는다. 송신 전용이고 모든 처리를 폴링으로
수행하기 때문이다. 상태 LED는 사용하지 않으며 A3와 A5는 예비 핀으로 남긴다.
D10~D13에는 MCP2515 이외의 장치를 연결하지 않는다.

### 로거 Uno(B)와 MCP2515

로거 Uno(B)는 다음처럼 연결한다.

| 로거 Uno 핀 | MCP2515 |
|---|---|
| 5 V | VCC(모듈 정격 확인) |
| GND | GND |
| D10 | CS |
| D11 | MOSI/SI |
| D12 | MISO/SO |
| D13 | SCK |

로거도 INT 없이 RX 상태를 폴링한다. Ubuntu PC의 USB로 로거 Uno(B)에 전원을
공급할 수 있다. Arduino IDE에서 제어 Uno와 로거 Uno의 포트를 혼동하지 않도록
하나씩 연결해 포트 번호를 먼저 기록한다.

### CAN 버스와 공통 GND

```text
제어측 CAN-H ---------------- 로거측 CAN-H
제어측 CAN-L ---------------- 로거측 CAN-L
제어측 GND   ---------------- 로거측 GND
```

두 CAN 노드의 CAN-H끼리, CAN-L끼리 연결하고 비절연 TJA1050 모듈의 신호 기준을
맞추기 위해 GND도 공통으로 둔다. CAN-H와 CAN-L은 한 쌍으로 꼬고 모터·배터리
전원선과 떨어뜨린다. 이후 노드를 추가할 때는 스타 배선 대신 하나의 주 버스를
연장하고 분기선은 짧게 유지한다.

### 종단저항 확인

현재 두 노드가 버스의 양 끝이므로 양쪽 끝에 120 ohm 종단저항이 각각 하나씩
있어야 한다. 일부 MCP2515 모듈에는 저항이나 점퍼가 이미 있으므로 중복으로
추가하지 않는다.

모든 전원을 끈 상태에서 CAN-H와 CAN-L 사이를 측정한다.

| 측정값 | 판정 |
|---:|---|
| 약 60 ohm | 정상: 120 ohm 2개가 병렬 연결됨 |
| 약 120 ohm | 종단저항이 한쪽에만 있음 |
| 무한대에 가까움 | 종단저항 없음 또는 CAN선 단선 |
| 약 40 ohm | 종단저항이 3개 있음 |
| 0 ohm에 가까움 | CAN-H/CAN-L 단락 의심 |

약 60 ohm과 CAN-H/CAN-L의 단락 없음이 확인된 뒤 전원을 인가한다.

## 코드 활성화 순서

후륜 D6/D7 물리 배선과 RC 시험이 이미 완료됐다면 아래의 CAN 활성화 단계부터
진행한다. 첫 CAN 통신 확인은 ROS와 액추에이터 출력을 끄고 사람용 시리얼을 켠다.
CAN 수신이 확인된 뒤 RC/ROS 실차 조합으로 단계적으로 복귀한다.

```cpp
#define BROON_ENABLE_ROS 0
#define BROON_ENABLE_HUMAN_SERIAL 1
#define BROON_ENABLE_CAN_TELEMETRY 1
#define BROON_ENABLE_ACTUATOR_OUTPUTS 0
```

1. 차량의 24 V 모터 전원을 차단한다.
2. 후륜 PWM D6, DIR D7과 D10~D13 MCP2515 배선을 다시 확인한다.
3. `BuildOptions.h`에서 `BROON_ENABLE_CAN_TELEMETRY`를 `1`로 변경한다.
4. Arduino IDE에서 `BROON_T870_Uno_Controller.ino`를 연다.
5. `도구 > 보드 > Arduino Uno`와 제어 Uno(A)의 포트를 선택한다.
6. 검증(체크 표시)으로 컴파일한 뒤 업로드한다.
7. 차량 모터 전원은 계속 끈 채 로거 Uno(B)의 수신 여부부터 확인한다.

CAN 송신기는 20 ms마다 프레임 한 개만 시도하며 여섯 종류가 한 세트를 이룬다.
정상 조건의 완전한 CSV 행은 약 120 ms마다 생성된다. 순차 RC `pulseIn()` 때문에
실제 주기는 지연될 수 있다. 송신 버퍼가 비어 있지 않으면 재시도하며 기다리지 않고
즉시 프레임을 버려 차량 제어 루프를 우선한다.

CAN 초기화 실패, 상대 노드 전원 차단, ACK 부재 또는 버스 단선은 차량 명령이나
출력 허가를 바꾸지 않는다. 통신을 복구한 뒤 프레임이 다시 나오지 않으면 차량을
안전하게 정지하고 두 Uno를 재부팅한다.

## CAN 프레임

정확한 signal bit 위치와 단위는 [DBC](can/T870_CAN.dbc)에 정의되어 있다.

| ID | 이름 | 주요 데이터 |
|---:|---|---|
| `0x100` | STATUS | sequence, state, fault, RC/ROS, 안전 flags, uptime, protocol |
| `0x101` | DRIVE | 요청·전륜·후륜 PWM |
| `0x102` | STEERING | 목표 ADC, 실제 ADC, 허가 PWM |
| `0x103` | MOTION | 속도(0.01 km/h), 전륜 encoder delta, 교정 상태 |
| `0x104` | RC | 조향·스로틀·AUX 펄스와 유효 flags |
| `0x105` | TIMING | RC 읽기 시간, 누적 drop, EFLG, TEC, REC |

`sequence`가 같은 여섯 프레임만 하나의 완전한 샘플이다. 로거의 `complete=0`은
해당 sequence에서 프레임이 하나 이상 빠졌다는 뜻이며 주행 분석에서 제외하거나
결측치로 처리한다.

## 로거 Uno 사용법

1. Arduino IDE에서
   `logger/T870CanCsvLogger/T870CanCsvLogger.ino`를 연다.
2. 보드를 Arduino Uno로, 포트를 로거 Uno(B)로 선택해 업로드한다.
3. 시리얼 모니터를 `115200` baud로 열면 CSV 헤더와 행이 출력된다. 제어 Uno의
   포트를 열지 않도록 주의한다.
4. `protocol=1`, `complete=1`, 증가하는 `seq`를 확인한다.
5. `tx_dropped`, `can_eflg`, `can_tec`가 계속 증가하면 종단저항, bitrate,
   오실레이터, CAN-H/L, 상대 노드 전원을 확인한다.

정상 출력은 다음 헤더로 시작한다.

```text
logger_ms,complete,seq,protocol,state,fault,mode,status_flags,...
@CAN,2254036,0x102,8,3A01E803D2000000
```

`@CAN` 행은 `logger_ms`, 표준 CAN ID, DLC, 실제 데이터 바이트를 차례로 담은
원본 프레임이다. 통합 실행기는 이 행을 `can_frames.csv`로 분리하고, 일반 CSV 행은
자동진단용 `raw_can.csv`로 저장한다.

`# MCP2515_INIT_FAILED; retrying every 1000 ms`가 반복되면 로거 Uno와 MCP2515
사이의 5 V, GND, CS D10, MOSI D11, MISO D12, SCK D13 및 `8.000` 크리스털을
먼저 확인한다. 헤더만 출력되고 데이터 행이 없다면 제어 Uno의 CAN 활성화,
제어측 모듈 전원, CAN-H/CAN-L 극성 및 약 60 ohm 종단저항을 확인한다.

Arduino IDE의 독립 스케치 제약 때문에 로거 폴더에는 루트의
`T870CanProtocol.h`, `T870Mcp2515.h`와 동일한 복사본이 들어 있다. 프로토콜이나
드라이버를 수정할 때는 로거 복사본도 동일하게 갱신하고 파일 해시를 비교한다.

로거의 `logger_ms`는 로거 Uno 부팅 후 경과시간이다. Ubuntu의 실제 시각을 붙여
파일로 저장하려면 다음을 사용한다.

```bash
timedatectl
sudo timedatectl set-timezone Asia/Seoul
sudo timedatectl set-ntp true

ls /dev/ttyACM* /dev/ttyUSB*
python3 -m pip install --user -r logger/requirements.txt

cd /프로젝트/경로/Mando/arduino/BROON_T870_Uno_Controller
mkdir -p logs
python3 logger/save_serial_csv.py \
  --port /dev/ttyACM0 \
  --output logs/t870_can_test01.csv
```

Arduino IDE 시리얼 모니터는 저장 스크립트를 실행하기 전에 닫는다. 종료는
`Ctrl+C`다. `--output`을 생략하면 파일명은
`t870_can_YYYYMMDD_HHMMSS.csv`가 되며 첫 열
`host_time_iso`에는 Ubuntu가 설정한 현지 시각과 UTC offset이 기록된다. 다른 장치가
시리얼 포트를 사용 중이면 로거가 열리지 않는다. 단순 저장기도 원본 프레임을
`t870_can_YYYYMMDD_HHMMSS_frames.csv`에 별도로 저장한다.

```text
host_time_iso,logger_ms,complete,seq,...
2026-08-23T14:32:18.351+09:00,15230,1,42,...
```

- `host_time_iso`: Ubuntu가 행을 받은 실제 시각
- `logger_ms`: 로거 Uno 부팅 후 경과시간
- `complete`: 같은 sequence의 여섯 프레임 수신 완료 여부

`host_time_iso`는 제어 Uno의 정확한 송신 시각이 아니라 Ubuntu 수신 시각이므로
프레임 조립과 USB 전송 지연이 포함된다. 일반 주행 로그에는 사용할 수 있지만
정밀 센서 동기화 시각으로 사용하지 않는다.

### 캡처 종료 후 자동진단까지 한 번에 실행

팀원이 주행 당일 별도 명령을 여러 번 입력하지 않도록 다음 통합 실행기를 권장한다.

```bash
python3 logger/capture_and_diagnose.py \
  --port /dev/serial/by-id/실제_로거_Uno_이름 \
  --name rc_normal_01
```

`CSV header received; recording rows.`가 출력된 뒤 주행하고, 차량을 정지한 다음
`Ctrl+C`를 누르면 `runs/날짜_시간_시험명/`에 다음 결과가 함께 생성된다.

- `can_frames.csv`: CAN ID·DLC·데이터 바이트 원문
- `raw_can.csv`: 여섯 메시지를 조립한 자동진단용 텔레메트리
- `dbc_decoded_frames.csv`: DBC로 독립 해석한 프레임
- `dbc_verification_summary.md`: DBC와 로거 해석값 일치 여부
- `diagnosis/diagnosis_summary.md`: 규칙 기반 고장진단 결과

Windows에서는 `--port COM7`처럼 입력한다. 주행 전에 차량 없이 10초 검증하는
방법과 당일 확인표는 `logger/QUICK_CAPTURE_KO.md`를 따른다.

### 기존 원본 프레임을 DBC로 다시 검증

```bash
python3 can/decode_can_frames.py \
  --input runs/시험폴더/can_frames.csv \
  --telemetry runs/시험폴더/raw_can.csv
```

검증기는 `T870_CAN.dbc`로 각 프레임을 독립 해석한 뒤, 같은 sequence에 대해 로거
Uno가 생성한 `raw_can.csv` 값과 신호별로 비교한다. 보고서의 `PASS`는 두 해석
구현이 같은 CAN 바이트를 동일하게 해석했다는 의미이며 차량 자체의 무고장을
보증하지는 않는다.

시리얼 권한 오류가 발생하면 아래 명령을 실행하고 Ubuntu에서 로그아웃한 뒤 다시
로그인한다.

```bash
sudo usermod -aG dialout $USER
```

## 주행 후 자동 고장진단

저장된 CSV를 사람이 직접 모든 열과 시간대를 비교하지 않아도 되도록 규칙 기반
자동진단기를 제공한다. 딥러닝과 머신러닝은 사용하지 않으며 원본 CSV를 수정하지
않는다.

```bash
cd Mando/arduino/BROON_T870_Uno_Controller/fault_diagnosis

python3 analyze_t870_log.py \
  --input ../logs/t870_can_test01.csv \
  --output reports/t870_can_test01 \
  --no-plot
```

`diagnosis_summary.md`, `diagnosis_events.csv`, `diagnosis_data.csv`가 생성된다.
matplotlib를 설치하고 `--no-plot`을 빼면 `diagnosis_plot.png`도 생성된다. 검출 범위,
임계값 및 시험 방법은 `fault_diagnosis/README_KO.md`를 따른다.

## 단계별 시험과 합격 기준

### 1. 모터 전원 차단 시험

1. 차량 24 V 모터 전원을 차단한다.
2. 두 Uno와 두 MCP2515에 5 V 전원을 인가한다.
3. 로거에서 `complete=1`, `protocol=1`, 증가하는 `seq`를 확인한다.
4. RC 수신기를 켜고 `rc_steer_us`, `rc_throttle_us`, `rc_aux_us`가 리모컨에
   따라 변하는지 확인한다.

이 단계에서 연속된 완전한 행이 출력되면 SPI와 기본 CAN 버스 연결을 승인한다.

### 2. 리프트 상태 구동 시험

1. 바퀴를 지면에서 완전히 띄운다.
2. 물리 24 V 차단 스위치를 즉시 누를 수 있게 준비한다.
3. 모터 전원을 켜고 RC 저출력부터 전륜·후륜·조향을 작동한다.
4. CSV의 `drive_req_pwm`, `front_pwm`, `rear_pwm`, `steer_target_adc`,
   `steer_actual_adc`, `steer_pwm`, `speed_kph`, `encoder_delta`가 실제 동작과
   일치하는지 확인한다.
5. CAN 활성화 전후 RC 방향, 출발, 정지, 램프 및 원격정지 동작이 같아야 한다.

### 3. CAN 고장 분리 시험

바퀴를 띄운 상태에서 로거 Uno 전원 차단과 CAN-H/L 단선을 각각 시험한다. CAN
통신이 사라져도 차량 RC 제어와 원격정지가 정상이어야 한다. CAN은 관측 전용이므로
CAN 고장 때문에 모터가 새로 구동되거나 기존 안전 정지가 해제되면 안 된다.

### 4. 주행 로그 시험

저속·짧은 구간부터 주행하고 CSV에서 다음을 확인한다.

- `complete=1` 행이 안정적으로 생성됨
- `seq`가 정상적으로 증가함
- `tx_dropped`가 지속적으로 빠르게 증가하지 않음
- `can_eflg`, `can_tec`, `can_rec`가 지속 증가하지 않음
- 조향 ADC와 전륜 속도가 실제 차량 동작과 일치함

## 문제 해결표

| 증상 | 우선 확인 항목 |
|---|---|
| `MCP2515_INIT_FAILED` 반복 | 모듈 5 V/GND, D10~D13 SPI, CS, `8.000` 크리스털 |
| CSV 헤더만 나오고 행이 없음 | 제어 Uno CAN 활성화, 제어측 모듈 전원, CAN-H/L 극성 |
| `complete=0` 빈번 | 프레임 누락, 배선 노이즈, 종단저항, 전원 불안정 |
| `can_tec` 증가 | 상대 노드 ACK 부재, 로거 전원, bitrate, 종단저항 |
| `can_eflg` 증가 | CAN 오류 상태; H/L 단락·극성·접촉 불량 확인 |
| Ubuntu 포트 권한 오류 | `dialout` 그룹 추가 후 로그아웃/로그인 |
| `/dev/ttyACM0` 없음 | `/dev/ttyACM*`, `/dev/ttyUSB*` 재확인 |
| CAN 복구 후 송신 재개 안 됨 | 차량 안전 정지 후 두 Uno 전원 재인가 |

## ROS와의 통합 조건

ROS 단독 빌드와 주행을 먼저 검증한다. 그 후 아래 조합을 컴파일하고 장시간 벤치
시험한다.

```cpp
#define BROON_ENABLE_ROS 1
#define BROON_ENABLE_HUMAN_SERIAL 0
#define BROON_ENABLE_CAN_TELEMETRY 1
```

Uno SRAM은 작기 때문에 컴파일 결과의 전역 SRAM뿐 아니라 30분 이상 rosserial+CAN
지속통신에서 리셋, ROS packet loss, CAN drop, RC loss 정지를 함께 관찰한다. CAN
프레임이 수신된다는 사실만으로 ROS+CAN 통합을 승인하지 않는다.
