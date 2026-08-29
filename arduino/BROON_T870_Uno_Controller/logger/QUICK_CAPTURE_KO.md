# T870 주행 당일 빠른 CSV 캡처·자동진단

이 절차는 로거 Uno의 CAN 원본 프레임과 해석된 텔레메트리를 각각 CSV로 저장한다.
캡처가 끝나면 규칙 기반 고장진단과 DBC 독립 재해석·교차검증까지 자동으로 수행한다.
제어 Uno의 RC/ROS 동작에는 관여하지 않는다.

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

- `can_frames.csv`: 실제 CAN ID, DLC, 8바이트 원문이 보존된 프레임 CSV
- `raw_can.csv`: 여섯 CAN 메시지를 합친 자동진단용 텔레메트리 CSV
- `dbc_decoded_frames.csv`: `T870_CAN.dbc`로 독립 재해석한 프레임
- `dbc_verification_summary.md`: DBC 해석값과 로거 Uno 해석값의 교차검증 결과
- `run_metadata.json`: 포트, 시작·종료 시각, 행·프레임 수, 마지막 sequence
- `diagnosis/diagnosis_summary.md`: 사람이 읽는 자동진단 결과
- `diagnosis/diagnosis_events.csv`: 검출 이벤트 목록
- `diagnosis/diagnosis_data.csv`: 원본과 파생 진단 데이터

`can_frames.csv`에는 `0x100`부터 `0x105`까지의 ID와 `data_hex`가 있어야 한다.
`raw_can.csv` 첫 줄은 `host_time_iso,logger_ms,complete,seq,...`로 시작한다.
`dbc_verification_summary.md`가 `PASS`이고 메타데이터의 `row_count`와
`can_frame_count`가 모두 0보다 크면 원본 수신·해석·교차검증이 정상이다.

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
두 CSV가 닫히고 자동진단과 DBC 교차검증이 바로 실행된다. ROS 주행은 이름을
`ros_normal_01`처럼 바꾸면 된다. 그래프까지 원하면 명령 끝에 `--plot`을 붙인다.
DBC 처리를 일시적으로 생략해야 할 때만 `--skip-dbc`를 붙인다.

이미 저장한 `can_frames.csv`만 다시 DBC로 검증할 수도 있다.

```bash
python3 can/decode_can_frames.py \
  --input runs/시험폴더/can_frames.csv \
  --telemetry runs/시험폴더/raw_can.csv
```

## 주행 당일 30초 확인표

1. 로거 Uno 포트 확인
2. Arduino 시리얼 모니터 닫기
3. 위 한 줄 명령 실행
4. `CSV header received` 확인
5. 주행 시작
6. 차량 정지 후 `Ctrl+C`
7. 출력된 `diagnosis_summary.md`와 `dbc_verification_summary.md` 열기

`Ctrl+C`를 누르기 전까지 CSV는 행마다 즉시 flush되므로 예기치 않은 중단이 있어도
이미 받은 행은 대부분 보존된다. 다만 로그가 실제 고장을 확정하는 것은 아니며,
보고서의 원인은 점검 우선순위 후보로 사용한다.
