# Localization 문서

실행 방법은 [패키지 README](../README.md), 구성요소와 연결 관계는
[아키텍처](architecture.md)를 기준으로 확인합니다. 설정값의 원본은 `config/` YAML입니다.

| 문서 | 담당 내용 |
|---|---|
| [아키텍처](architecture.md) | 현재 실행 노드, 데이터 흐름, TF와 출력 소유권 |
| [설정 가이드](configuration.md) | 설정 파일의 역할, 실행 모드와 변경 방법 |
| [TF 프레임](tf_frames.md) | 차량 기준점, 장착값, TF 검증 |
| [GPS 품질과 재연결](gps_quality_and_recovery.md) | GPS 승인 조건, covariance, 장기 단절 복구 |
| [상태와 출력 승인](status_and_recovery.md) | Supervisor 우선순위, 상태 enum, Output Gate |
| [센서 시각](sensor_timing.md) | clock_ready, 측정 시각 보간, 지연 GPS 처리 |
| [CalibratedIMU](calibrated_imu.md) | GNSS 전진 직진 구간을 이용한 최초 yaw 정합 |

과거 실험의 설정과 결과는 작업공간의 `rosbag/`, `.codex-runtime/`에 보관합니다.
운영 기본값과 실험별 override는 구분합니다. 정리 전 소스는
작업공간 `snapshots/1차빌드완료/`에 보존되어 있습니다.
