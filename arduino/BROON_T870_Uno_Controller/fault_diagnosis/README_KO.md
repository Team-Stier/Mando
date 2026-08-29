# T870 CAN 로그 자동 고장진단기

주행 후 `save_serial_csv.py`가 저장한 CSV를 규칙 기반으로 분석한다. 딥러닝,
강화학습 및 실시간 차량제어 기능은 사용하지 않는다. 원본 CSV는 읽기만 하며 별도
폴더에 진단 결과를 만든다.

## 검출 범위

- 제어기 fault와 재시작
- 조향센서 범위 이탈, 추종오차, 무응답, 헌팅 가능성
- 구동 피드백 없음, 과속, 무명령 움직임, 전·후륜 PWM 불일치
- RC 채널 무효, 펄스 범위 이탈, 원격정지
- CAN incomplete, sequence 누락, 수신 공백, 송신 drop, EFLG, TEC, REC

후륜 엔코더, 배터리 전압, 모터 전류 및 온도 데이터가 없으므로 해당 부품의 고장을
확정하지 않는다. 보고서의 원인은 우선 점검할 후보로 사용한다.

## 실행

기본 분석과 텍스트·CSV 출력에는 Python 표준 라이브러리만 필요하다.

```bash
cd Mando/arduino/BROON_T870_Uno_Controller/fault_diagnosis

python3 analyze_t870_log.py \
  --input ../logs/t870_can_test01.csv \
  --output reports/t870_can_test01 \
  --no-plot
```

그래프도 생성하려면 matplotlib를 설치하고 `--no-plot`을 빼고 실행한다.

```bash
python3 -m pip install --user -r requirements-optional.txt

python3 analyze_t870_log.py \
  --input ../logs/t870_can_test01.csv \
  --output reports/t870_can_test01
```

결과 폴더에는 다음 파일이 생성된다.

| 파일 | 내용 |
|---|---|
| `diagnosis_summary.md` | 사람이 읽는 종합 진단보고서 |
| `diagnosis_events.csv` | 이벤트 시작·종료·심각도·원인·근거 |
| `diagnosis_data.csv` | 원본 열과 파생 특징·이벤트 번호 |
| `diagnosis_plot.png` | 속도·PWM·조향·CAN 추이(선택) |

`confidence_pct`는 규칙의 데이터 직접성을 설명하기 위한 고정 점수이며 실제 고장
확률로 해석하지 않는다.

## 임계값 조정

`config/thresholds.json`에서 지속시간과 임계값을 조정한다. 초기값은 현재 Arduino
설정의 조향 무응답 1초, 구동 활성 PWM 60, ROS 측정속도 제한 15.0 km/h를
반영했다. 구동 무응답 1.5초 규칙은 펌웨어 fault 설정이 아니다. 현재 펌웨어의
`kDriveNoFeedbackTimeoutMs`는 `0`으로 비활성화되어 있으며, 1.5초는 주행 후 로그에서
후보 이상을 찾기 위한 임시 진단 기준이다. 첫 정상 주행 로그를 확보한 뒤 실제
노이즈와 응답에 맞춰 조정한다.

## 자동시험

```bash
python3 -m unittest discover -s tests -v
```

합성 로그로 정상, 조향 무응답, 구동 무응답, 과속, CAN 오류, RC 채널 무효 및
보고서 생성을 검사한다.
