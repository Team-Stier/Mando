# T870 실차 CAN 검증 증거 패키지

이 폴더는 T870 CAN 텔레메트리와 규칙 기반 로그 진단기를 실제 차량 데이터로
검증하기 위한 기록을 보관한다. 차량 제어 코드는 이 폴더의 파일을 읽지 않는다.

## 기본 원칙

1. 시험계획은 시험 전에 작성하고 로컬 Git 커밋으로 고정한다.
2. 원본 CSV는 수정하지 않는다. 복사본 또는 분석 결과만 변경한다.
3. 시험에 사용한 코드 커밋과 설정 파일을 함께 보관한다.
4. 최초 임계값 결과와 수정 후 결과를 서로 다른 폴더에 보관한다.
5. 정상·이상 여부는 현장 시험기록을 기준으로 판정하고 진단기 결과와 비교한다.
6. 구현하지 않았거나 측정할 수 없는 항목은 PASS로 추정하지 않는다.

## 2026-08-30 시험 패키지

`2026-08-30` 폴더는 2026-08-29에 시험 전 작성한 최초 실차 검증 패키지다.
실제 시험일이 바뀌더라도 이 계획서를 덮어쓰지 말고 실행기록에 실제 날짜를 적는다.

| 경로 | 용도 |
|---|---|
| `01_plan/T870_CAN_VALIDATION_PLAN.md` | 사전 요구사항, Test Case와 합격 기준 |
| `02_config/` | 시험 커밋과 사용 설정 스냅샷 |
| `03_raw/` | 수정하지 않은 원본 CAN CSV와 해시 |
| `04_reports_before/` | 최초 임계값으로 생성한 진단 결과 |
| `05_reports_after/` | 임계값 수정 후 진단 결과 |
| `06_test_record/test_execution_record.csv` | 현장 실행 시각·조건·판정 기록 |
| `07_final/T870_CAN_VALIDATION_REPORT_TEMPLATE.md` | 최종 결과 및 회귀시험 보고서 양식 |

## 시험 직전 증거 고정

아래 명령은 `validation/2026-08-30` 폴더에서 실행한다. 파일명에 저장된 실제 설정이
드러나도록 `*_used` 접미사를 사용한다.

```bash
git rev-parse HEAD > 02_config/git_commit.txt
git status --short > 02_config/git_status_before_test.txt
cp ../../BuildOptions.h 02_config/BuildOptions_used.h
cp ../../ControllerConfig.h 02_config/ControllerConfig_used.h
cp ../../fault_diagnosis/config/thresholds.json \
  02_config/thresholds_before.json
```

`git_status_before_test.txt`가 비어 있지 않으면 어떤 변경을 사용했는지 실행기록에
반드시 설명한다. 가능하면 깨끗한 커밋 상태에서 시험한다.

## 원본 로그 보존

로거가 생성한 CSV는 `03_raw`에 저장하고 분석 전에 SHA-256 해시를 만든다.

```bash
sha256sum 03_raw/*.csv > 03_raw/SHA256SUMS.txt
```

해시 생성 후 원본 CSV를 편집하지 않는다. 시험자 메모는
`06_test_record/test_execution_record.csv`에 작성한다.

## 최초 분석과 회귀시험

최초 임계값 분석 예시는 다음과 같다.

```bash
python3 ../../fault_diagnosis/analyze_t870_log.py \
  --input 03_raw/t870_ros_normal_01.csv \
  --output 04_reports_before/t870_ros_normal_01 \
  --no-plot
```

오탐 또는 미탐 근거가 있을 때만 `thresholds.json`을 수정한다. 수정 이유와 전후값을
최종 보고서에 기록하고 모든 원본 로그를 다시 분석해 `05_reports_after`에 저장한다.
한 로그만 좋아졌다는 이유로 승인하지 않고 기존 정상·이상 Test Case가 모두 유지되는지
확인한다.
