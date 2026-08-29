# 보관된 수정 전 진단 결과

아래 값은 각 주행 직후 `diagnosis/diagnosis_events.csv`에 보관된 결과다. 과거 로그는
서로 다른 개발 시점의 규칙으로 분석되었으므로, 97건 전체 감소를 이번 한 번의
로직 변경 성과로 주장하지 않는다. 이번 변경의 직접 비교 대상은 마지막 정상
RC+CAN 로그의 조향 급변 오탐 3건이다.

| 로그 | 행 | 보관 이벤트 수 | 주요 이벤트 |
|---|---:|---:|---|
| `180632_idle_smoke` | 14 | 40 | 초기 incomplete·sample gap·remote stop 반복 |
| `202402_rc_signal_check` | 68 | 2 | immediate stop, RC steer pulse range |
| `203602_rc_drive_01` | 182 | 45 | CAN/RC 초기값, 조향 안전범위·급변·추종 |
| `212146_rc_steering_tuned_01` | 220 | 1 | sensor jump 1 |
| `212435_rc_steering_tuned_02` | 550 | 6 | safe range 2, sensor jump 3, tracking 1 |
| `215119_rc_raw_dbc_01` | 570 | 3 | 정상 도달 구간 sensor jump 3(FP) |

마지막 로그의 세 이벤트는 현재 샘플의 `steer_pwm=0`만 보고 판정했지만, 직전
샘플에는 `|PWM|=210`이 있었고 실제 ADC가 직전 목표 오차를 줄이는 방향으로
이동했다. 현장 동작과 데이터 관계상 센서 고장이 아니라 정상 조향 도달로 분류했다.
