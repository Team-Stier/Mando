# TF 프레임과 외부 파라미터

## 좌표계

기본 계약은 2D ENU와 REP-103 관례입니다.

- `base_link`: +X 전방, +Y 좌측, +Z 위
- `base_link` 원점: 차량 뒷바퀴 축 중심(`rear_axle_center`)
- 위치: m
- ROS 회전·각속도: rad, rad/s
- `tf_configuration.yaml.rotation_rpy_deg`: degree, roll/pitch/yaw 순서
- frame 이름: 선행 `/`와 공백 없음

## TF 트리와 유일 소유자

```text
map
 `-- odom                 Global EKF 동적 TF
      `-- base_link       Local EKF 동적 TF
           |-- imu_link   검증된 정적 TF
           |-- gps_link   검증된 정적 TF
           `-- laser_link 검증된 정적 TF
```

| Transform | 유형 | 유일 발행자 | 현재 기본 상태 |
|---|---|---|---|
| `map -> odom` | dynamic | `odometry_gps_lidar_global_ekf` | Global EKF 실행 시 발행 |
| `odom -> base_link` | dynamic | `imu_encoder_local_ekf` | Local EKF 실행 시 발행 |
| `base_link -> imu_link` | static | `localization_static_tf_publisher` | X=+0.25 m만 확인, 나머지 미측정·비활성 |
| `base_link -> gps_link` | static | 같은 노드 | `unmeasured`, 비활성 |
| `base_link -> laser_link` | static | 같은 노드 | X=+1.05 m만 확인, 나머지 미측정·비활성 |

AMCL은 launch에서 `tf_broadcast=false`를 강제하고 Xsens 설정은 `pub_transform=false`입니다. 다른 URDF/static publisher가 같은 child를 이미 소유한다면 패키지 정적 TF를 동시에 활성화하면 안 됩니다.

## 시작 시 동적 소유자 검증

`tf_configuration.yaml.dynamic_transforms`는 변환 값을 발행하는 설정이 아니라 소유권 선언입니다.

```yaml
dynamic_transforms:
  - parent_frame: map
    child_frame: odom
    owner_node: odometry_gps_lidar_global_ekf
    source: robot_localization_global
  - parent_frame: odom
    child_frame: base_link
    owner_node: imu_encoder_local_ekf
    source: robot_localization_local
```

`StaticTransformPublisher`는 시작하면서 다음을 검사합니다.

1. `schema_version: 1`, `strict_validation: true`인지 확인합니다.
2. `base_link_reference`가 비어 있지 않고 축이 `forward/left/up`인지 확인합니다.
3. static과 dynamic 선언을 합친 트리에서 중복 child, self edge, 잘못된 frame, 순환 구조와 비유한 수치를 거부합니다.
4. `safety_and_tf.launch`가 넘긴 `expected_local_owner`, `expected_global_owner`와 YAML의 두 `owner_node`가 각각 일치하는지 확인합니다.
5. 조건을 모두 만족한 정적 TF만 `/tf_static`으로 발행합니다.

따라서 launch에서 EKF node name을 바꾸면 `tf_configuration.yaml.owner_node`도 같은 이름으로 바꿔야 합니다. 불일치하면 정적 TF 노드가 시작을 중단합니다.

> 이 검증은 launch 인자와 YAML 선언의 정합성 및 설정된 트리 구조를 검사합니다. ROS master의 모든 `/tf` publisher를 탐색해 런타임 중복 발행자를 자동 검출하는 기능은 아니므로 `tf_monitor`, `view_frames`로 실제 발행자도 확인해야 합니다.

## 정적 TF 발행 조건

```yaml
static_transforms:
  - parent_frame: base_link
    child_frame: imu_link
    translation_m: [0.25, 0.0, 0.0]
    rotation_rpy_deg: [0.0, 0.0, 0.0]
    enabled: false
    calibration_state: unmeasured
    measured_at: ""
    source: ""
```

현재 IMU와 LiDAR의 X만 각각 +0.25 m, +1.05 m로 확인됐습니다. Y/Z와 RPY의 0은 자리표시자이며 완전한 장착 위치로 간주하지 않습니다. 다음 두 조건을 모두 만족해야 발행합니다.

1. `enabled: true`
2. `calibration_state: measured` 또는 `verified`

`enabled=true`여도 `unmeasured`이면 발행하지 않습니다. 모든 항목이 비활성이면 노드는 경고를 내고 정적 TF 없이 계속 실행합니다.

## 측정·활성화 절차

1. 차량 운동 모델이 사용하는 `base_link` 원점을 물리적으로 표시합니다.
2. 같은 +X/+Y/+Z 축 기준으로 각 센서 기준점의 위치를 m로 측정합니다.
3. 제조사 frame 정의를 확인하고 `base_link`에서 sensor frame으로 향하는 roll/pitch/yaw를 degree로 측정합니다.
4. 날짜, 측정 도구와 원본 기록 위치를 `measured_at`, `source`에 기록합니다.
5. 먼저 `calibration_state: measured`, `enabled: true`로 바꾸고 정지 상태에서 TF 방향을 확인합니다.
6. 실제 직진·좌회전·주행 데이터로 축과 부호를 검증한 뒤 `verified`로 승격합니다.

## 현재 부분 측정값

`base_link`를 뒷바퀴 축 중심으로 두고 차량 전방을 +X로 정의했을 때 현재 받은 값은 다음과 같습니다.

| 센서 | X | Y | Z | RPY | 발행 상태 |
|---|---:|---:|---:|---|---|
| IMU | +0.25 m | 미측정 | 미측정 | 미측정 | 비활성 |
| 2D LiDAR | +1.05 m | 미측정 | 미측정 | 미측정 | 비활성 |
| GPS 안테나 | 미측정 | 미측정 | 미측정 | 미측정 | 비활성 |

Y/Z 또는 회전각을 0으로 가정해 활성화하지 않습니다. 특히 2D LiDAR의 높이·pitch·roll 오차는 지도 정합을 직접 왜곡하므로 전체 6DoF를 측정한 뒤 발행해야 합니다.

### IMU

- 2026-08-30 auto-scan에서 `/dev/ttyUSB0`, serial `DB8GG04M`, device ID `03889250` (`MTi-3-8A7G6`), 115200 baud를 확인했습니다.
- `sensor_msgs/Imu` 약 100.95 Hz, `imu_link`, timestamp 존재를 확인했고 raw covariance 세 배열은 모두 0이었습니다. 현재 `measured` override가 정규화 출력에 양의 covariance를 생성하므로 형식상 EKF 입력은 통과할 수 있지만, 정지 noise floor라 운영 정확도는 아직 검증되지 않았습니다.
- 정지 시 중력 방향과 `linear_acceleration` 축을 확인합니다.
- 좌회전 시 ROS 기준 `angular_velocity.z > 0`인지 확인합니다.
- `header.frame_id == imu_link`인지 확인합니다.
- orientation을 융합하므로 실제 장착 yaw와 orientation covariance를 함께 검증합니다.

### GPS

- `gps_link`는 안테나 phase center를 기준으로 측정합니다.
- `NavSatFix.header.frame_id == gps_link`인지 확인합니다.
- 현재 GPS 직접 투영 게이트는 `base_link <-> gps_link` TF로 lever arm을 보정하지 않습니다. 정적 TF 기록만으로 GPS map pose가 자동으로 차체 기준점으로 바뀌지 않으므로 정밀 보정 기능을 별도로 검증해야 합니다.
- 2D 융합에서 높이를 사용하지 않더라도 안테나 Z와 datum 고도는 기록합니다.

### 2D LiDAR

- scan 평면이 차량 수평면과 평행한지 확인합니다.
- 0° 방향과 차량 +X가 일치하는지 확인합니다.
- roll/pitch 오차는 2D map matching을 체계적으로 왜곡하므로 위치와 각도를 함께 측정합니다.

## 런타임 검증

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link imu_link
rosrun tf tf_echo base_link gps_link
rosrun tf tf_echo base_link laser_link
rosrun tf tf_monitor
rosrun tf view_frames
rosrun rqt_tf_tree rqt_tf_tree
```

확인 항목:

- `map -> odom`, `odom -> base_link` 발행자가 각각 하나인지
- 하나의 child에 둘 이상의 parent가 없는지
- 정적 TF는 `/tf_static`, 동적 TF는 갱신 timestamp로 보이는지
- 센서 메시지 frame에서 `map`까지 트리가 끊기지 않는지
- RViz Fixed Frame `map`에서 LaserScan과 차량 위치가 같은 지도에 올바르게 겹치는지
