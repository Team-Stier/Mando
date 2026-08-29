# 시험 설정 스냅샷

시험 직전에 다음 파일을 이 폴더에 저장한다.

- `git_commit.txt`: `git rev-parse HEAD` 결과
- `git_status_before_test.txt`: 시험 직전 작업트리 상태
- `BuildOptions_used.h`: 제어 Uno에 업로드한 빌드 설정
- `ControllerConfig_used.h`: 시험 차량 제어·교정 설정
- `thresholds_before.json`: 최초 분석 임계값
- `thresholds_after.json`: 수정한 경우에만 추가

RC, ROS 또는 CAN 단계마다 빌드 설정이 달라지면 하나의 파일을 덮어쓰지 말고
`BuildOptions_CAN01_used.h`, `BuildOptions_RC_used.h`,
`BuildOptions_ROS_used.h`처럼 분리한다.

2026-08-29 RC+CAN 실행에 대해서는 다음 증거를 보관한다.

- `EVIDENCE_CHAIN_KO.md`: 계획·시험·분석 커밋과 설정 해시
- `BuildOptions_RC_used.h`: RC+CAN 빌드 설정 스냅샷
- `thresholds_before.json`: 분석에 사용한 임계값

`ControllerConfig.h`는 `70cf7ab`의 blob과 현재 파일이 동일함을 Git blob과
SHA-256으로 확인했다. 상세 값은 `EVIDENCE_CHAIN_KO.md`에 있다.
