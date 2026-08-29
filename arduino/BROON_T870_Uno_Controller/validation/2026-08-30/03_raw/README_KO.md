# 원본 로그 보관

이 폴더에는 로거가 직접 생성한 CSV를 수정 없이 저장한다. 분석용으로 열을 추가하거나
행을 삭제하지 않는다. 시험 메모는 `../06_test_record/test_execution_record.csv`에
남긴다.

권장 파일명은 시험계획의 Test Case를 따른다.

```text
t870_can_poweroff_01.csv
t870_rc_stationary_01.csv
t870_rc_drive_01.csv
t870_rc_steering_01.csv
t870_ros_drive_01.csv
t870_ros_steering_01.csv
t870_rc_loss_01.csv
t870_ros_timeout_01.csv
```

수집이 끝나면 다음 명령으로 무결성 해시를 만든다.

```bash
sha256sum *.csv > SHA256SUMS.txt
```

현재 원본 로그는 프로젝트의 `runs/` 폴더에 원본 그대로 보존하며 Git에는 올리지
않는다. `SHA256SUMS.txt`에는 실제 상대경로와 해시를 기록했다. 핵심 최종 캡처는
`20260829_215119_rc_raw_dbc_01`이고, 570개 완전 샘플과 3,422개 원본 CAN 프레임을
포함한다.
