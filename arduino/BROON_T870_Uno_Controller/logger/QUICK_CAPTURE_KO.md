# T870 주행 당일 빠른 CSV 캡처·자동진단

이 절차는 로거 Uno의 CAN 데이터를 CSV로 저장하고, 캡처가 끝나는 즉시 규칙 기반
고장진단 보고서를 생성한다. 제어 Uno의 RC/ROS 동작에는 관여하지 않는다.

## 주행 전 한 번만 준비

프로젝트 루트에서 다음을 실행한다.

```bash
cd Mando/arduino/BROON_T870_Uno_Controller
python3 -m pip install --user -r logger/requirements.txt
```

그래프까지 필요하면 다음 선택 패키지도 설치한다.

```bash
python3 -m pip install --user -r fault_diagnosis/requirements-optional.txt
```

Arduino IDE의 시리얼 모니터와 시리얼 플로터는 반드시 닫는다. 한 포트는 한
프로그램만 열 수 있다. Windows에서는 장치 관리자나 Arduino IDE로 로거 Uno의
`COM` 번호를 확인한다. Ubuntu에서는 재연결에도 이름이 잘 유지되는
`/dev/serial/by-id/` 경로 사용을 권장한다.

```bash
ls -l /dev/serial/by-id/
```

## 차량 없이 10초 사전 확인

두 Uno와 CAN 모듈에 전원만 넣고 모터 전원은 차단한다. 현재 로거가 `COM7`이라면
Windows PowerShell에서 다음을 실행한다.

```powershell
cd C:\path\to\Mando\arduino\BROON_T870_Uno_Controller
python logger\capture_and_diagnose.py --port COM7 --name idle_smoke --duration 10
```

Ubuntu 예시는 다음과 같다.

```bash
cd /path/to/Mando/arduino/BROON_T870_Uno_Controller
python3 logger/capture_and_diagnose.py \
  --port /dev/serial/by-id/실제_로거_Uno_이름 \
  --name idle_smoke \
  --duration 10
```

성공하면 `runs/날짜_시간_idle_smoke/` 아래에 다음이 생긴다.

- `raw_can.csv`: 실제 시각이 붙은 원본 CAN CSV
- `run_metadata.json`: 포트, 시작·종료 시각, 행 수, 마지막 sequence
- `diagnosis/diagnosis_summary.md`: 사람이 읽는 자동진단 결과
- `diagnosis/diagnosis_events.csv`: 검출 이벤트 목록
- `diagnosis/diagnosis_data.csv`: 원본과 파생 진단 데이터

`raw_can.csv` 첫 줄이 `host_time_iso,logger_ms,complete,seq,...`로 시작하고 데이터
행이 있으며, 메타데이터의 `row_count`가 0보다 크면 CSV 추출은 정상이다.

통합 실행기는 이미 CAN을 수신 중인 로거 Uno를 방해하지 않도록 COM 포트를 열 때
DTR/RTS 자동 리셋을 비활성화한다. 따라서 Arduino IDE 시리얼 모니터에서 정상
데이터를 확인한 뒤 모니터만 닫고 실행해도 로거 펌웨어가 다시 시작되지 않는다.

## 실제 주행 때 사용할 명령

시험 조건을 알아볼 수 있게 `--name`만 바꿔 한 번 실행한다.

```bash
python3 logger/capture_and_diagnose.py \
  --port /dev/serial/by-id/실제_로거_Uno_이름 \
  --name rc_normal_01
```

명령 실행 후 `CSV header received; recording rows.`가 보이면 주행을 시작한다.
주행이 끝나 차량을 안전하게 정지한 뒤 터미널에서 `Ctrl+C`를 한 번 누른다. 그러면
CSV가 닫히고 자동진단이 바로 실행된다. ROS 주행은 이름을 `ros_normal_01`처럼
바꾸면 된다. 그래프까지 원하면 명령 끝에 `--plot`을 붙인다.

## 주행 당일 30초 확인표

1. 로거 Uno 포트 확인
2. Arduino 시리얼 모니터 닫기
3. 위 한 줄 명령 실행
4. `CSV header received` 확인
5. 주행 시작
6. 차량 정지 후 `Ctrl+C`
7. 출력된 `diagnosis_summary.md` 경로 열기

`Ctrl+C`를 누르기 전까지 CSV는 행마다 즉시 flush되므로 예기치 않은 중단이 있어도
이미 받은 행은 대부분 보존된다. 다만 로그가 실제 고장을 확정하는 것은 아니며,
보고서의 원인은 점검 우선순위 후보로 사용한다.
