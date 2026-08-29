# 최초 임계값 분석 결과

`02_config/thresholds_before.json`을 사용한 각 원본 로그의 진단 결과를 Test Case별
하위 폴더에 저장한다. 결과를 본 뒤 이 폴더의 파일을 수정하지 않는다.

예시:

```text
04_reports_before/
└─ t870_ros_drive_01/
   ├─ diagnosis_summary.md
   ├─ diagnosis_events.csv
   └─ diagnosis_data.csv
```

실행 결과 요약은 `RC_CAN_REGRESSION_BEFORE.md`에 있다. 실제 상세 CSV는 원본과
같은 `runs/<실행명>/diagnosis/`에 보존한다.
