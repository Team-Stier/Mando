# Stier RTAB-Map SLAM workspace

ROS1 Noetic 기반 1/5 차량용 2D LiDAR SLAM 기초 워크스페이스다. 센서 업체별
USB/serial 드라이버는 정규화 입력 토픽 앞의 adapter로 분리하며, 이 저장소는
wheel odometry, IMU 기반 scan leveling, RTK FIX gate, local EKF, RTAB-Map
mapping/localization, 지도 편집·불변 release, 임시 장애물 layer를 담당한다.

현재 코드는 하드웨어 독립 합성 입력으로 검증할 수 있지만, 실제 센서 adapter와
물리 calibration은 아직 완료된 것으로 간주하지 않는다. 실차 측정이 승인될 때까지
`calibration_required:=true`가 기본값이며 `real_mode:=true` 실행은 중단된다.

## Workspace layout

- `stier_slam_core`: 계산 정책, ROS node, 지도 release/editor/runtime guard
- `stier_slam_bringup`: YAML, RTAB-Map INI, launch, RViz
- `stier_slam_test_support`: 실제 장비를 열지 않는 결정론적 합성 supplier
- `docs/hardware-calibration-checklist.md`: 실차 전 필수 측정·승인표
- `docs/rosbag-mapping-workflow.md`: record, replay, mapping, edit/release,
  localization 운용 절차

## Dependency bootstrap

조회만 수행하는 preflight:

```bash
./scripts/bootstrap_dependencies.sh --check
```

이 명령은 `dpkg-query`만 사용하며 누락된 모든 package를 출력하고 하나라도 없으면
exit 1이다. 설치가 명시적으로 승인된 경우에만 다음을 실행한다.

```bash
./scripts/bootstrap_dependencies.sh --install
```

`--install`은 고정되고 정렬된 package 목록을 먼저 출력한 뒤 그 목록만
`sudo apt-get install`에 전달한다. 다른 flag, 인자 누락, trailing argument는 exit 2다.

## Build, test, and install

ROS Noetic의 system Python을 사용한다. Conda 또는 user-site `setuptools`가 Ubuntu의
Catkin용 patched `setuptools`를 가리면 `--install-layout=deb` 단계가 실패할 수 있다.
전역 Python 설정은 바꾸지 말고 이 workspace 명령에만 다음 환경을 적용한다.

```bash
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSETUPTOOLS_DEB_LAYOUT=ON
catkin_make run_tests \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSETUPTOOLS_DEB_LAYOUT=ON
catkin_test_results --verbose build/test_results
catkin_make install \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSETUPTOOLS_DEB_LAYOUT=ON
```

새 shell에서 install-space import가 devel/source가 아닌
`install/lib/python3/dist-packages`를 사용하는지 확인한다.

```bash
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
source install/setup.bash
/usr/bin/python3 -c \
  'import stier_slam_core; print(stier_slam_core.__file__)'
```

하드웨어 없이 처리 파이프라인을 확인하는 bounded smoke:

```bash
source devel/setup.bash
timeout --signal=INT --kill-after=5s 40s \
  rostest stier_slam_bringup synthetic_pipeline_smoke.test
```

이 smoke는 wheel/local odometry, leveled cloud, RTK gate, released demo map,
sensor health, live-obstacle 생성·만료와 TF ownership을 검증한다. 테스트 전용 identity
`map -> odom` fixture를 사용하고 `/rtabmap`이 없음을 명시적으로 검사하므로,
RTAB-Map loop closure, 실제 `map -> odom`, DB 저장·재로드, localization 정확도 또는
실차 calibration의 증거가 아니다.

2026-08-08 Ubuntu/ROS Noetic, system Python (`PYTHONNOUSERSITE=1`,
`/usr/bin/python3`) 환경에서 `rostest stier_slam_bringup
synthetic_pipeline_smoke.test`를 5회 연속 통과했다. 아래는 마지막 대표
실행의 실수신 주기이며, synthetic fixture의 성능 baseline일 뿐 실차 성능
보장이 아니다.

| Topic | 관측 rate (Hz) |
|---|---:|
| `/clock` | 19.987 |
| `/tf` | 15.501 |
| `/slam/odometry/wheel` | 19.991 |
| `/slam/odometry/local` | 15.501 |
| `/slam/gnss/fix_accepted` | 19.993 |
| `/slam/scan/leveled_points` | 19.993 |
| `/slam/diagnostics` | 5.000 |
| `/slam/diagnostics/rtk_gate` | 19.993 |
| `/slam/live_obstacles/points` | 23.561 |
| `/slam/live_obstacles/grid` | 23.538 |

같은 날 dependency preflight의 정확한 누락 목록은 `nodejs`,
`ros-noetic-rtabmap-ros`, `ros-noetic-tf2-sensor-msgs`다. RTAB-Map과
tf2 sensor package 부재 때문에 실제 mapping/localization, loop closure, `/grid_map`
export, 실차 rosbag·경사로 field acceptance는 아직 실행 근거가 없다.
별도로 현재 `node` executable은 사용 가능해 JavaScript syntax test는 통과했지만,
재현 가능한 editor 검증 환경을 위한 Debian `nodejs` package는 누락 상태다.

## Normalized ROS contract

업체별 adapter는 다음 이름, 단위, frame, timestamp 계약으로만 publish한다.
토픽 이름의 중앙 기준은
[`src/stier_slam_bringup/config/topics.yaml`](src/stier_slam_bringup/config/topics.yaml)이며,
자체 Python node는 `/stier_slam/topics/...` ROS parameter를 읽어 Publisher와 Subscriber를
생성한다. RTAB-Map remap, local EKF 입력과 rosbag record 목록은 ROS1 설정 제약 때문에
각 launch/config에 남아 있으며 launch contract test가 중앙 기준과의 일치를 검사한다.

| Input topic | Type | Contract |
|---|---|---|
| `/slam/input/wheel_encoder` | `sensor_msgs/JointState` | 좌·우 wheel 이름, 누적 `position` rad 또는 `velocity` rad/s, source stamp |
| `/slam/input/scan_raw` | `sensor_msgs/LaserScan` | 유효한 source stamp와 LiDAR frame |
| `/slam/input/imu` | `sensor_msgs/Imu` | orientation quaternion, angular velocity rad/s |
| `/slam/input/gnss/fix` | `sensor_msgs/NavSatFix` | finite WGS84 좌표와 covariance |
| `/slam/input/gnss/rtk_status` | `diagnostic_msgs/DiagnosticStatus` | `message`가 `FIX`, `FLOAT`, `NO_FIX` 중 하나 |
| `/slam/input/vehicle_speed` | `geometry_msgs/TwistWithCovarianceStamped` | `linear.x` m/s, 전진 양수, source stamp와 covariance |
| `/slam/input/steering_angle` | `std_msgs/Float64` | road-wheel angle rad, 좌회전 양수 |
| `/slam/input/camera/front/image_raw` | `sensor_msgs/Image` | 기록·health 대상이며 현재 SLAM 입력은 아님 |

| Output topic | Type | Frame/time contract |
|---|---|---|
| `/slam/odometry/wheel` | `nav_msgs/Odometry` | `odom -> base_link`, speed source stamp 보존, TF 없음 |
| `/slam/odometry/local` | `nav_msgs/Odometry` | local EKF의 planar `odom -> base_link` estimate |
| `/slam/scan/leveled_points` | `sensor_msgs/PointCloud2` | `base_link`, scan source stamp, XYZ m |
| `/slam/gnss/fix_accepted` | `sensor_msgs/NavSatFix` | gate를 통과한 FIX의 원 source stamp/좌표 보존 |
| `/slam/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | RTAB-Map의 `map` pose |
| `/slam/map/released` | `nav_msgs/OccupancyGrid` | 불변 release의 `map` grid |
| `/slam/live_obstacles/points` | `sensor_msgs/PointCloud2` | 측정 시각의 `map` 임시 point |
| `/slam/live_obstacles/grid` | `nav_msgs/OccupancyGrid` | bounded `map` layer, cell 값 `-1/100` |
| `/slam/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ROS time 기준 sensor health |
| `/slam/diagnostics/rtk_gate` | `diagnostic_msgs/DiagnosticStatus` | RTK 수용·거절 이유와 count |

모든 stamped 입력은 같은 시간 기준의 양수·유한·단조 증가 source stamp를 보존해야
한다. rosbag 재생에서는 `/use_sim_time:=true`와 단일 `/clock` publisher를 사용한다.
실센서 supplier와 replay supplier를 동일한 normalized topic에 동시에 연결하지 않는다.

`wheel_encoder_adapter_node`는 `velocity`를 우선 사용하고, 없으면 연속된 `position`을
source timestamp로 차분한다. `encoder.yaml`의 wheel radius와 방향 부호를 적용해
`/slam/input/vehicle_speed`로 변환하며, 이 출력은 기존 Ackermann wheel odometry와
local EKF 경로에서 그대로 사용한다.

## TF ownership and map safety

```text
map --RTAB-Map--> odom --local EKF--> base_link
                                      |-- lidar_link
                                      |-- imu_link
                                      |-- gps_link
                                      `-- camera_link
```

- RTAB-Map만 `map -> odom`을 publish한다.
- local EKF만 `odom -> base_link`를 publish한다. Wheel odometry node는 TF를 publish하지
  않는다.
- 측정된 static extrinsic의 owner는 한 개의 URDF/static TF supplier뿐이어야 한다.
- raw capture와 release는 수정하거나 덮어쓰지 않는다. 편집은 workspace에서 수행하고
  새 release ID로만 publish한다.
- static edit는 release map에 rasterize된다. 주행 중 live obstacle은 별도 bounded
  layer이며 기본 0.75 s TTL 후 사라지고 release에 기록되지 않는다. 현재 이 layer의
  planner/controller consumer는 구현되어 있지 않다.
- 2D map/EKF는 고도를 추정하지 않는다. 경사로에서는 IMU roll/pitch로 scan을
  `base_link` cloud로 level하고 yaw는 leveling에서 제외한다.

전체 운용 명령과 장애 복구는
[`docs/rosbag-mapping-workflow.md`](docs/rosbag-mapping-workflow.md), 실차 전 승인 조건은
[`docs/hardware-calibration-checklist.md`](docs/hardware-calibration-checklist.md)를 따른다.
