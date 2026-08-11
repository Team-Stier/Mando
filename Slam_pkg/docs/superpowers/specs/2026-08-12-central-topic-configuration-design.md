# 중앙 ROS 토픽 설정 설계

## 목적

SLAM 입력·처리·출력 토픽의 표준 이름을 `topics.yaml` 한 파일에서 확인하고 변경할 수
있게 한다. 자체 Python node는 하드코딩 대신 ROS parameter를 읽고, 외부 ROS node의
remap과 EKF 설정은 contract test로 중앙 설정과 일치함을 보장한다.

## 구조

- `config/topics.yaml`에 `topics.inputs`, `topics.processed`, `topics.outputs`를 둔다.
- mapping, localization, synthetic demo launch가 이 파일을 global parameter namespace에
  한 번 load한다.
- 자체 Python node는 `/stier_slam/topics/...` parameter를 필수로 읽는다.
- 메시지 type은 코드 계약이므로 기존 Python 코드에 유지한다.
- RTAB-Map의 remap과 `robot_localization`의 `odom0`/`imu0`는 ROS1 XML/YAML 제약 때문에
  기존 설정 파일에 남기되, launch contract test가 `topics.yaml` 값과 비교한다.
- record launch는 중앙 설정과 같은 토픽을 유지하고 contract test로 일치시킨다.

## 대상 토픽

- 입력: LiDAR, IMU, GNSS fix, RTK status, vehicle speed, steering angle, front camera
- 처리: wheel/local odometry, leveled cloud, accepted GNSS
- 출력: localization pose, released map, diagnostics, RTK diagnostics, live obstacle cloud/grid

## 안전 조건

- 기본 토픽 이름과 메시지 type을 바꾸지 않는다.
- parameter 누락 시 임의 토픽으로 연결하지 않고 node 시작을 실패시킨다.
- 실제 센서 supplier와 rosbag supplier를 동시에 같은 normalized input에 연결하지 않는다.
- 사용자 미추적 `docs/system_architecture.*`는 수정하거나 커밋하지 않는다.

## 검증

- 먼저 중앙 설정 및 node parameter 사용을 요구하는 실패 테스트를 추가한다.
- 각 node unit test에서 변경된 topic parameter가 실제 publisher/subscriber에 적용되는지
  확인한다.
- launch contract test로 YAML, RTAB-Map remap, EKF input과 record 목록의 일치를 확인한다.
- Python compile, unit test 및 Catkin launch contract test를 실행한다.
