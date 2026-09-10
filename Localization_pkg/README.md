# Localization 작업공간

ROS1 Noetic용 실센서 localization 소스입니다. 패키지 동작과 공개 인터페이스는
[`mando_localization` 문서](src/mando_localization/README.md)를 기준으로 합니다.

## 저장소에서 빌드

저장소 루트에서 실행합니다. ROS Noetic과 패키지의 `package.xml`에 선언한
의존성이 설치되어 있어야 합니다.

```bash
cd Localization_pkg
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
catkin_make run_tests -j1
catkin_test_results build/test_results
```

`src/ublox`는 [KumarRobotics/ublox](https://github.com/KumarRobotics/ublox)의
`4f107f3b82135160a1aca3ef0689fd119199bbef` 기반 소스와 로컬 GNSS 측정 시각
수정을 함께 보관합니다. NAV-PVT UTC를 사용하는 헤더 시각, 유효하지 않은 UTC의
발행 차단, 시각 진단과 회귀 검사를 포함합니다. 원본 BSD 라이선스는
[`src/ublox/LICENSE`](src/ublox/LICENSE)에 있습니다. 일반 u-blox 설치본으로
대체하면 이 작업공간의 시각 계약과 달라질 수 있습니다.

## 실행 경로

패키지 문서의 `/home/stier/Mando_1-5/localization`은 기존 현장 작업공간입니다.
이 저장소에서 실행하려면 위 빌드 후 현재 작업공간을 명시합니다.

```bash
export MANDO_LOCALIZATION_WS="$PWD"
rosrun mando_localization localization_command.sh
# rosbag 기록까지 시작할 때:
# rosrun mando_localization localization_record_command.sh
```

실센서 실행 전 `config/*_driver.yaml`과 `udev/` 문서의 장치 식별자를 확인합니다.
`localization_command.sh`는 사용 가능한 센서로 실행하며, 기록 명령은 기본적으로
필수 센서 수신을 확인합니다. 일반 `bringup.launch`에서 AMCL을 사용할 때는
실제 지도와 해당 좌표계에 맞는 설정이 별도로 필요합니다.

`rddf/`에는 현재 뷰어와 초기 방향 검사가 사용하는 용인 경로 원본을 그대로
포함합니다. 기본 초기 yaw는 `yongin_1_right.csv`의 첫 행을 사용하므로 차량을
해당 출발 방향에 배치해야 합니다. 다른 방향에서 시작하면
`initialize_heading:=false` 또는 적절한 `initial_heading_config`를 사용합니다.

빌드 산출물, rosbag, 수집 기록, 로컬 분석 결과와 스냅샷은 Git에 포함하지 않습니다.
합성 ROS 검사와 빌드 통과는 실차 정확도, GNSS/PPS 동기 또는 closed-loop 주행
검증을 의미하지 않습니다.
