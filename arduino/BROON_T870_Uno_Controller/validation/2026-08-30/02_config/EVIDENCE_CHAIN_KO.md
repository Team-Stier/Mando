# 시험 코드·설정 증거 연결

## Git 기준점

| 구분 | 커밋 | 시각(KST) | 의미 |
|---|---|---|---|
| 시험계획 기준선 | `42c6fe6` | 2026-08-29 16:38:59 | 최초 실차 로그보다 먼저 검증계획이 Git에 고정됨 |
| 최종 RC 시험 직전 제어기 | `70cf7ab` | 2026-08-29 21:11:23 | 조향 교정·복구 로직과 RC 빌드 설정 |
| 원본 CAN/DBC 기능 보존 | `ace8dd7` | 2026-08-29 21:58:23 | 21:51~21:53 캡처에 사용한 원본 프레임·DBC 기능을 시험 직후 커밋 |
| 조향 오탐 수정 | `473b682` | 2026-08-29 | 직전 PWM 정상 도달 응답을 급변에서 제외하고 자동시험 2개 추가 |

최종 캡처는 `ace8dd7` 커밋 5분 전에 실행되었으므로 시험 중 작업트리가 깨끗했다는
증거는 없다. 대신 `run_metadata.json`, 원본 파일 SHA-256, 시험 직후 커밋과 DBC
교차검증 결과를 함께 보존한다. 이 한계를 숨기지 않고 최종 보고서에 기록한다.

## 사용 설정 동일성

- `BuildOptions.h` Git blob: `6dd756f30405f409ec0b04c5594609a16a573d71`
- `ControllerConfig.h` Git blob: `4f797ce376fc4219787ff0319e890fb710a43353`
- 현재 파일은 `70cf7ab`의 각 blob과 동일하다.
- `BuildOptions.h` SHA-256: `239695b4d6ddb0e56e99cbfccc6b3d74ed11c5e3e86bc65f2120f9481dc4297d`
- `ControllerConfig.h` SHA-256: `f62377279b1b7e4e8e81461e199157d8c924cf5d0aeb66ad7535f5abcc0deb26`
- 실행 진단 임계값 SHA-256: `d6d86bc20d2eaf38a54c690d8d5eb4be75a62e8a3573ddb8d3f576adaca75e51`
- 보관 JSON SHA-256: `0b36ea8fada322be8333c5ba39a510f86fef92ec5e445bcc0f6e7621c60b142e`
- 두 JSON은 줄바꿈 형식 때문에 파일 해시는 다르지만 파싱한 키·값은 동일하다.

RC 캡처 설정은 액추에이터 출력 1, ROS 0, 사람용 Serial 1, CAN 텔레메트리 1,
래치 고장 0이다. 전체 설정 내용은 같은 폴더의 `BuildOptions_RC_used.h`와
`thresholds_before.json`에서 확인한다. `ControllerConfig.h`는 위 커밋의 파일을
정본으로 사용한다.
