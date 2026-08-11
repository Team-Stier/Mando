# Wheel Encoder Adapter 설계

## 목적

표준 ROS `sensor_msgs/JointState`로 전달되는 좌·우 rear wheel encoder를 차량 전진속도
`geometry_msgs/TwistWithCovarianceStamped`로 변환해 기존 Ackermann wheel odometry와
Local EKF에 연결한다.

## 입력 계약

- Topic: `/slam/input/wheel_encoder`
- Type: `sensor_msgs/JointState`
- `header.stamp`: 실제 encoder 측정 시각
- `name`: `rear_left_wheel`, `rear_right_wheel`
- `position`: wheel 기준 누적 회전각 rad
- `velocity`: wheel 기준 rad/s. 두 값이 모두 있으면 우선 사용한다.

`velocity`가 비어 있으면 연속된 `position`과 timestamp 차이로 rad/s를 계산한다. 첫
position sample은 기준점만 만들고 출력하지 않는다. 이름 누락·중복, 배열 길이 불일치,
비유한 값, 시간 역행 및 설정된 최대 wheel speed 초과는 출력을 만들지 않는다.

## 출력 계약

- Topic: `/slam/input/vehicle_speed`
- Type: `geometry_msgs/TwistWithCovarianceStamped`
- `header.stamp`: encoder source timestamp를 그대로 보존
- `header.frame_id`: `base_link`
- `twist.twist.linear.x`: `(left_rad_s + right_rad_s) * wheel_radius_m / 2`
- covariance: `encoder.yaml`의 36개 finite 값

## 구성과 launch

`encoder.yaml`은 wheel radius, joint 이름, 좌우 방향 부호, 최대 wheel angular speed,
output frame과 covariance를 보관한다. `real_mode=true`이면서 calibration이 남아 있으면
node 시작을 중단한다. adapter는 mapping/localization launch에만 추가하고 synthetic
fixture에는 추가하지 않아 기존 synthetic vehicle-speed publisher와 충돌하지 않게 한다.

## 검증

- pure Python 계산은 velocity 우선, position 차분, 좌우 평균, 부호, 시간·값 오류를 단위
  테스트한다.
- ROS wrapper는 topic parameter, message type, source timestamp와 covariance를 검사한다.
- launch contract는 config load, required node, record allowlist와 중앙 topic schema를 검사한다.
- 기존 Local EKF 입력 `/slam/input/vehicle_speed` 이후 경로는 변경하지 않는다.
