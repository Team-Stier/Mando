# 조향 오탐 수정 후 회귀시험

- 진단 코드 커밋: `473b682`
- 임계값 변경: 없음
- 자동 단위시험: 15/15 PASS
- 실제 로그 회귀: 6/6 분석 완료, 새 이벤트 유형 0건
- 마지막 정상 RC+CAN 로그: `STEERING_SENSOR_JUMP` 3건 → 0건

| 로그 | 수정 전 보관 이벤트 | 현재 이벤트 | 현재 주요 이벤트 | 회귀 판정 |
|---|---:|---:|---|---|
| `180632_idle_smoke` | 40 | 3 | immediate stop, remote stop, RC invalid | PASS(과거 초기 로그) |
| `202402_rc_signal_check` | 2 | 1 | immediate stop | PASS(과거 초기 로그) |
| `203602_rc_drive_01` | 45 | 9 | safe range 9 | PASS(관찰사항 유지) |
| `212146_rc_steering_tuned_01` | 1 | 1 | sensor jump 1, 직전 PWM 0 | PASS(검출 유지) |
| `212435_rc_steering_tuned_02` | 6 | 3 | safe range 2, tracking 1 | PASS(관찰사항 유지) |
| `215119_rc_raw_dbc_01` | 3 | 0 | 없음 | PASS(FP 제거) |

`212146`의 646 ADC 급변은 직전 PWM이 0이고 직전 목표오차도 0이므로 이번 정상
도달 예외에 해당하지 않는다. 실제 센서 접촉 또는 당시 조작 기록이 없어 고장으로
확정하지 않고 판정 보류로 남긴다. `212435`의 안전범위·추종 이벤트도 원본 사실을
삭제하지 않고 과거 튜닝 관찰사항으로 보존한다.
