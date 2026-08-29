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
