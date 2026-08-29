# T870 CAN 로그 자동 고장진단기

주행 후 `save_serial_csv.py`가 저장한 CSV를 규칙 기반으로 분석한다. 딥러닝,
강화학습 및 실시간 차량제어 기능은 사용하지 않는다. 원본 CSV는 읽기만 하며 별도
폴더에 진단 결과를 만든다.

## 검출 범위

- 제어기 fault와 재시작
- 조향센서 전기·안전 범위 이탈, 순간 급변, 고착 가능성
- 조향 추종오차, 무응답, 반대방향 응답, 헌팅 가능성
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

## 조향 중심 진단

조향 진단은 `steer_target_adc`, `steer_actual_adc`, `steer_pwm`의 시간 관계를
비교한다. RC/ROS 모드와 관계없이 같은 CAN 신호를 사용하지만, 출력 허용과 명령
유효 flag가 켜지고 즉시정지가 아닌 구간에서만 액추에이터 응답 이상을 추정한다.
따라서 의도적인 RC 원격정지 중 큰 목표오차가 남아도 무응답으로 판정하지 않는다.

| 코드 | 판정 개념 |
|---|---|
| `STEERING_SENSOR_RANGE` | ADC가 전기적 유효범위 1~1022를 이탈 |
| `STEERING_SAFE_RANGE` | 실차 안전 명령범위 200~1000을 허용오차 이상 지속 이탈 |
| `STEERING_SENSOR_JUMP` | 정상 PWM 응답으로 설명되지 않는 급격한 ADC 변화 발생 |
| `STEERING_TRACKING_ERROR` | 목표가 안정된 뒤에도 PWM 출력 중 목표오차가 지속됨 |
| `STEERING_NO_RESPONSE` | 목표가 안정된 상태에서 PWM 대비 ADC 변화가 없음 |
| `STEERING_REVERSE_RESPONSE` | 실제 ADC가 목표와 반대방향으로 계속 이동 |
| `STEERING_SENSOR_STUCK` | 충분한 시간 동안 PWM이 출력되고 목표가 변하지만 실제 ADC 범위가 고정됨 |
| `STEERING_HUNTING` | 목표가 일정한데 PWM 방향과 실제 ADC가 반복 진동 |

추종오차는 80 ADC에서 진입하고 40 ADC 이하에서 해제하는 서로 다른 기준을 사용해
경계값 부근의 반복 판정을 줄인다. 캡처 시작 시 첫 CAN sequence가 불완전한 경우는
주행 중 프레임 누락으로 판정하지 않는다. 제어기보다 늦게 로거가 연결되면서 생기는
ACK 부재 복구를 고장으로 오인하지 않도록 최초 5초의 EFLG/TEC/REC는 준비구간으로
제외하며, 오류가 그 이후에도 남아 있으면 정상적으로 진단한다.

## 임계값 조정

`config/thresholds.json`에서 지속시간과 임계값을 조정한다. 초기값은 현재 Arduino
설정의 조향 무응답 1초, 구동 활성 PWM 60, ROS 측정속도 제한 15.0 km/h를
반영했다. 구동 무응답 1.5초 규칙은 펌웨어 fault 설정이 아니다. 현재 펌웨어의
`kDriveNoFeedbackTimeoutMs`는 `0`으로 비활성화되어 있으며, 1.5초는 주행 후 로그에서
후보 이상을 찾기 위한 임시 진단 기준이다. 첫 정상 주행 로그를 확보한 뒤 실제
노이즈와 응답에 맞춰 조정한다.

정차 실측에서 완전한 여섯 프레임 sequence 간격이 약 0.647초까지 늘어나는 것을
확인해 이벤트 병합 간격을 0.8초, CAN 수신공백 경고를 1.0초로 설정했다. 조향센서
급변 250 ADC/0.25초 규칙은 2026-08-29 정상 조향 로그를 반영하여, 유효한 PWM
방향으로 목표 오차가 감소하는 변화는 정상 응답으로 제외한다. 센서 고착은 1.5초
창 안에서 활성 PWM이 최소 1초 지속된 경우에만 판단한다. 반대방향 응답 0.6초와
나머지 기준은 다음 RC·ROS 주행 로그로 계속 검증한다.

권장 검증 순서는 다음과 같다.

1. 정상 RC 주행에서 중앙·좌·우·복귀 구간과 조작 시각을 기록한다.
2. 같은 조건을 ROS 모드에서 기록한다.
3. 정상 로그의 HIGH 오탐을 확인하고 임계값 근거를 남긴다.
4. 합성 fault 로그로 무응답·역응답·급변·고착·헌팅 검출을 회귀시험한다.
5. 규칙 수정 전후 이벤트 수, 오탐 수와 검출시간을 비교한다.

## 자동시험

```bash
python3 -m unittest discover -s tests -v
```

합성 로그로 정상, 정지상태 조향 오탐 방지, 조향 무응답·역응답·급변·안전범위
이탈·고착·헌팅, 구동 무응답·과속, CAN 오류, RC 채널 무효 및 보고서 생성을
검사한다.
