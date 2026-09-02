# BROON T870 Arduino Uno 전환 설계

## 목표

현재 Mega 전용 하위 제어기를 Arduino Uno에서 컴파일하고 센서·RC 무출력 시험을 할 수 있도록 전환한다.

## 핀맵

- 엔코더 A/B: D2/D3
- 조향: PWM D5, DIR D4
- 전륜: PWM D9, DIR D8
- 후륜: PWM D11, DIR D10
- RC 조향/스로틀/AUX: A0/A1/A2
- 조향 포텐시오미터: A4
- 상태 LED: D13
- D0/D1은 USB Serial 또는 rosserial을 위해 비워 둔다.

## RC 입력 방식

Mega의 PORT F와 Timer5는 제거한다. Uno의 A0~A2는 PORT C의 PCINT8~10이므로 `PCINT1_vect`에서 세 입력의 상승·하강 에지를 동시에 캡처한다. 펄스폭은 `micros()`로 계산하고, 메인 루프에서는 atomic snapshot을 읽어 기존 100 ms timeout 및 A1 throttle-cut 로직에 전달한다.

## 안전 경계

액추에이터 출력, ROS, 조향 교정은 기존 기본값대로 비활성 상태를 유지한다. 핀맵 및 RC 입력 전환은 안전 상태 머신·조향 가드·구동 인터록의 의미를 바꾸지 않는다.

## 검증

`arduino:avr:uno` 대상 전체 스케치 컴파일을 경고 수준 `all`로 실행한다. 빌드 후 전역 SRAM이 Uno 2048바이트 안에 충분한 여유를 두는지, 출력 잠금이 0인지, Mega 전용 Timer5/PINF 심볼이 남아 있지 않은지 검사한다.
