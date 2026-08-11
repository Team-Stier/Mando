# Rosbag Mapping, Map Release, and Localization Workflow

이 절차는 현재 checkout의 launch/CLI 계약을 그대로 사용한다. 경로 인자는 가능한 한
절대 경로를 사용한다. `raw/`, `releases/`는 불변 artifact이고 mapping session과
localization runtime bundle은 그 밖의 새 디렉터리여야 한다.

## 0. 범위와 안전 조건

- 이 저장소에는 센서 모델별 USB/serial driver가 없다. Vendor adapter는 향후
  `/slam/input/*` normalized contract 앞에 추가하고 core launch와 분리한다.
- `record.launch`는 supplier, health node, EKF, map writer를 시작하지 않는다. 이미
  publish 중인 allowlist만 LZ4 rosbag으로 기록한다.
- 실차 adapter와 rosbag player를 같은 normalized topic에 동시에 연결하지 않는다.
- Replay에서는 bag의 recorded `/tf`를 기본적으로 재생하지 않는다. Local EKF와
  RTAB-Map이 각각 `odom -> base_link`, `map -> odom`의 유일한 owner여야 한다.
- 실제 차량은
  [hardware calibration checklist](hardware-calibration-checklist.md)의 독립 승인이
  끝나기 전까지 `real_mode:=true calibration_required:=false`로 실행하지 않는다.
- 운전·mapping 중 map 편집 UI를 조작하지 않는다. 정차한 뒤 별도 담당자가 localhost
  UI를 사용한다.

먼저 dependency와 system-Python build를 확인한다.

```bash
./scripts/bootstrap_dependencies.sh --check
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSETUPTOOLS_DEB_LAYOUT=ON
source devel/setup.bash
```

`--check`가 누락 package를 출력하면 설치 또는 해당 기능의 external blocker를 먼저
해결한다. 특히 실제 mapping/localization은 `rtabmap_slam` 없이는 시작할 수 없다.

## 1. Record normalized inputs

센서 adapter와 단일 static-extrinsic supplier를 먼저 시작한 뒤 topic을 확인한다.

```bash
rostopic type /slam/input/scan_raw
rostopic type /slam/input/imu
rostopic type /slam/input/wheel_encoder
rostopic type /slam/input/gnss/fix
rostopic type /slam/input/gnss/rtk_status
rostopic type /slam/input/vehicle_speed
rostopic type /slam/input/steering_angle
rostopic info /tf_static
```

Type, unit, frame, timestamp는 README의 normalized contract와 일치해야 한다. 출력
directory를 준비하고 real sensor 기록을 시작한다.

```bash
RECORD_DIR="$HOME/rosbags/stier_slam"
mkdir -p "$RECORD_DIR"
roslaunch stier_slam_bringup record.launch \
  output_name:="$RECORD_DIR/kcity_run01" \
  use_sim_time:=false
```

현재 선언된 record launch args는 `real_mode`, `database_path`, `map_yaml`,
`use_sim_time`, `rviz`, `output_name`이다. 이 중 recorder 동작에는 `output_name`이,
global clock parameter에는 `use_sim_time`이 사용된다. 나머지는 현재 compatibility용
reserved args이며 driver, DB, map server 또는 RViz를 시작하지 않는다.

이 launch의 exact allowlist는 다음과 같다.

```text
/slam/input/wheel_encoder
/slam/input/scan_raw
/slam/input/imu
/slam/input/gnss/fix
/slam/input/gnss/rtk_status
/slam/input/vehicle_speed
/slam/input/steering_angle
/slam/input/camera/front/image_raw
/tf
/tf_static
/slam/diagnostics
/diagnostics
/clock
```

처리 결과(`/slam/odometry/*`, leveled cloud, accepted RTK, localization pose)는 의도적으로
기록하지 않아 replay에서 다시 계산한다. 정상 종료는 record terminal에서 `Ctrl-C`를
한 번 누르고 bag index flush가 끝날 때까지 기다린다. 강제 종료로 `.active` 파일이
남았다가 원본이 완전한 경우에만 다음으로 복구한다.

```bash
rosbag reindex /absolute/path/to/kcity_run01.bag.active
```

원본 `.active`를 삭제하거나 정상 `.bag` 위에 덮어쓰지 않는다.

## 2. Deterministic replay with one `/clock`

모든 real adapter를 종료한다. 다음 command는 recorded `/clock`과 recorded dynamic
`/tf`를 topic allowlist에서 제외하고 rosbag player가 하나의 `/clock`을 생성한다.
`/tf_static`에는 측정·승인된 sensor extrinsic만 있어야 한다.

```bash
BAG="/absolute/path/to/kcity_run01.bag"
rosparam set /use_sim_time true
rosbag play --clock --pause --keep-alive --wait-for-subscribers \
  "$BAG" --topics \
  /slam/input/scan_raw \
  /slam/input/imu \
  /slam/input/gnss/fix \
  /slam/input/gnss/rtk_status \
  /slam/input/vehicle_speed \
  /slam/input/steering_angle \
  /slam/input/camera/front/image_raw \
  /tf_static
```

Mapping launch가 준비된 뒤 player terminal에서 space를 눌러 재생한다. 다음 두
명령으로 clock과 normalized topic에 publisher가 하나뿐인지 확인한다.

```bash
rostopic info /clock
rostopic info /slam/input/scan_raw
```

Bag loop로 시간이 뒤로 가면 wheel odometry, freshness monitor, RTK gate와 live layer는
기존 epoch 상태를 유지하지 않아야 한다. 경고가 계속되거나 output이 복구되지 않으면
loop를 중단하고 새 launch로 재생한다.

## 3. Start a named mapping session

Mapping runtime은 course의 `raw/` 또는 `releases/` 안에 두지 않는다. Session ID는
한 번만 사용하며 session directory가 아직 존재하지 않는지 확인한다.

```bash
MAPPING_ROOT="$HOME/.ros/stier_slam_mapping"
MAPPING_SESSION="$MAPPING_ROOT/kcity_run01_mapping"
test ! -e "$MAPPING_SESSION"
roslaunch stier_slam_bringup mapping.launch \
  mapping_runtime_root:="$MAPPING_ROOT" \
  mapping_session_dir:="$MAPPING_SESSION" \
  real_mode:=false \
  calibration_required:=true \
  use_sim_time:=true \
  rviz:=true
```

`mapping.launch`의 현재 인자는 `real_mode`, `calibration_required`,
`mapping_runtime_root`, `mapping_session_dir`, `map_yaml`, `use_sim_time`, `rviz`다.
`map_yaml`은 현재 mapping graph에서 소비되지 않는 reserved arg이므로 입력 map을
로드한다고 가정하지 않는다.

실차 수동 mapping은 checklist 승인 뒤에만 다음 mode를 사용하며, 동일 normalized
topic에 rosbag player가 없어야 한다.

```bash
roslaunch stier_slam_bringup mapping.launch \
  mapping_runtime_root:="$MAPPING_ROOT" \
  mapping_session_dir:="$MAPPING_SESSION" \
  real_mode:=true \
  calibration_required:=false \
  use_sim_time:=false \
  rviz:=true
```

Mapping 중 확인할 핵심:

- `/slam/odometry/wheel`: `odom`, `base_link`, m/s와 rad/s
- `/slam/odometry/local`: local EKF의 planar odometry
- `/slam/scan/leveled_points`: scan source stamp를 보존한 `base_link` cloud
- `/slam/gnss/fix_accepted`: 신선하고 covariance/innovation gate를 통과한 `FIX`만
- `/tf`: local EKF의 `odom -> base_link`, RTAB-Map의 `map -> odom`
- `/slam/diagnostics`와 `/slam/diagnostics/rtk_gate`: `MISSING`, `STALE`,
  `TIME_REVERSED`, rejection reason 확인

RTK가 끊겨도 local EKF와 LiDAR mapping이 즉시 멈출 필요는 없지만, drift와 diagnostic을
기록한다. IMU orientation이 없거나 scan/IMU sync, metadata, tilt 검사가 실패하면
leveled scan은 fail closed로 drop된다.

## 4. Export and close the mapping session

차량 또는 paused bag을 완전히 정지시키고 마지막 update가 처리될 시간을 준다. RTAB-Map
node가 실행 중일 때 occupancy grid를 같은 mutable session으로 export한다.

```bash
rosrun map_server map_saver \
  -f "$MAPPING_SESSION/base" \
  map:=/grid_map
test -s "$MAPPING_SESSION/base.pgm"
test -s "$MAPPING_SESSION/base.yaml"
```

현재 launch는 RTAB-Map node를 namespace로 감싸지 않으므로 nodelet의 public latched
occupancy output은 `/grid_map`이다. Export 직전에 RTAB-Map dependency가 설치된
target에서 다음처럼 실제 graph를 확인한다.

```bash
rostopic type /grid_map
rostopic info /grid_map
```

Type은 `nav_msgs/OccupancyGrid`, publisher는 `/rtabmap`이어야 한다. 향후 launch에
namespace 또는 remap을 추가하면 이 경로도 함께 바뀌므로 추측하지 않는다. 실제
map_saver 성공은 RTAB-Map을 띄우지 않는 synthetic smoke의 검증 범위가 아니다.

Export 뒤 추가 주행을 하지 않고 mapping launch를 `Ctrl-C`로 정상 종료해 SQLite DB를
flush한다. 다음 파일이 모두 있어야 한다.

```bash
test -s "$MAPPING_SESSION/stier_slam_mapping.db"
test -s "$MAPPING_SESSION/base.pgm"
test -s "$MAPPING_SESSION/base.yaml"
```

Map export 또는 DB flush가 불확실하면 그 session을 release하지 말고 새 mapping ID로
다시 수행한다. 기존 session DB, INI, PGM/YAML을 부분 수정해 복구하지 않는다.

## 5. Capture, workspace, validate, release, and activate

Course directory와 각 ID를 정한다. ID는 경로가 아닌 단일 식별자이며 이미 사용한
capture/workspace/release ID를 재사용하지 않는다.

```bash
COURSE_DIR="$HOME/stier_maps/kcity_license_course"
CAPTURE_ID="capture_run01"
WORKSPACE_ID="workspace_run01"
RELEASE_ID="release_run01"

rosrun stier_slam_core map_release_cli.py init-capture \
  --course "$COURSE_DIR" \
  --capture "$CAPTURE_ID" \
  --db "$MAPPING_SESSION/stier_slam_mapping.db" \
  --map-yaml "$MAPPING_SESSION/base.yaml"

rosrun stier_slam_core map_release_cli.py init-workspace \
  --course "$COURSE_DIR" \
  --capture "$CAPTURE_ID" \
  --workspace "$WORKSPACE_ID"

rosrun stier_slam_core map_release_cli.py validate \
  --course "$COURSE_DIR" \
  --workspace "$WORKSPACE_ID"

rosrun stier_slam_core map_release_cli.py release \
  --course "$COURSE_DIR" \
  --workspace "$WORKSPACE_ID" \
  --release "$RELEASE_ID"

rosrun stier_slam_core map_release_cli.py activate \
  --course "$COURSE_DIR" \
  --release "$RELEASE_ID"
```

`init-capture`는 DB, PGM, YAML을 검증·snapshot하고 hash를 기록한다. 원본 mapping
session을 raw storage로 직접 사용하지 않는다. `raw/<capture_id>`와
`releases/<release_id>`는 생성 후 read-only로 취급하며 overwrite, symlink 교체,
수동 JSON/PGM 수정이 금지된다. Activation은 verified release를 가리키는
`active_release.json`을 atomic replace하지만 localization launch가 이 pointer를
자동으로 읽지는 않는다.

CLI의 `error:`/exit 2는 항상 새 ID를 쓰라는 의미가 아니다. 입력·hash·schema
오류는 commit 전에 원인을 먼저 고친다. 이미 다른 불변 artifact가 같은 ID를
사용하는 실제 collision이고 새 내용을 publish하려는 경우에만 새 ID를 사용한다.

CLI, API, editor 중 어느 경로든 `artifact publication durability is
indeterminate`, `active release durability is indeterminate` 또는 동등한
durability-indeterminate 상태를 반환하면 rename/replace가 이미 commit됐을
수 있다. 해당 ID/artifact/pointer를 삭제하거나 다른 bytes로 덮어쓰지 말고,
같은 ID의 hash·manifest·active pointer를 검증한 뒤 동일한 요청으로 retry한다.
일반 `activate` 실패에서도 release 자체를 새로 만들지 말고 원인을 해결한 후
기존 release ID로 activation을 다시 시도한다.

## 6. Localhost map editor

Workspace를 만든 뒤 다음 launch를 실행한다.

```bash
roslaunch stier_slam_bringup editor.launch \
  course_dir:="$COURSE_DIR" \
  workspace_id:="$WORKSPACE_ID" \
  host:=127.0.0.1 \
  port:=8765 \
unsafe_allow_nonlocal:=false
```

현재 editor args는 필수 `course_dir`, `workspace_id`와 선택 `web_root`, `host`, `port`,
`unsafe_allow_nonlocal`이다. 기본 `web_root`를 사용하고 nonlocal bind를 허용하지 않는다.

Browser에서 `http://127.0.0.1:8765`를 연다. Occupied/free polygon은
`static_overlay.json`, lane/stop-line 등 point/polyline은 `semantic_layer.json`에
map-frame meter 좌표로 저장된다. Save 전에 revision conflict를 해결하고 validate한 뒤
항상 새 release ID를 만든다.

Static edit는 현재 실행 중인 localization에 hot reload되지 않는다. 새 release를
명시적으로 선택한 다음 localization을 재시작해야 반영된다. 주행 중 자동 반영되는
것은 released map과 분리된 live-obstacle layer뿐이다. 기본 layer는 160 x 160 cell,
0.10 m/cell의 ego-centered grid이고 관측이 사라지면 0.75 s 후 expire한다. 이 layer는
raw/workspace/release를 수정하지 않는다. 현재 workspace에는 이 layer를 읽어 회피
제어하는 planner/controller가 없으므로, publish 또는 RViz 표시만으로 차량이 장애물을
회피한다고 간주하지 않는다.

## 7. Start immutable-release localization

선택한 release의 manifest/map과 그 release가 참조한 raw capture DB를 같은 조합으로
전달한다. `active_release.json`만 바꾸고 launch 인자를 그대로 두는 방식은 지원하지
않는다. Runtime root/bundle은 course directory 밖에 두고 launch마다 새 bundle ID를
사용한다.

```bash
LOCALIZATION_ROOT="$HOME/.ros/stier_slam_runtime"
RUNTIME_BUNDLE="$LOCALIZATION_ROOT/kcity_release_run01_attempt01"
test ! -e "$RUNTIME_BUNDLE"

roslaunch stier_slam_bringup localization.launch \
  database_path:="$COURSE_DIR/raw/$CAPTURE_ID/map.db" \
  map_yaml:="$COURSE_DIR/releases/$RELEASE_ID/map.yaml" \
  release_manifest:="$COURSE_DIR/releases/$RELEASE_ID/manifest.json" \
  runtime_root_dir:="$LOCALIZATION_ROOT" \
  runtime_bundle_dir:="$RUNTIME_BUNDLE" \
  real_mode:=false \
  calibration_required:=true \
  use_sim_time:=true \
  rviz:=true
```

실차 localization은 checklist 승인 뒤 `real_mode:=true`,
`calibration_required:=false`, `use_sim_time:=false`로 바꾸고 replay supplier를 끈다.
`localization.launch`의 필수 인자는 `database_path`, `map_yaml`, `release_manifest`이며
나머지 현재 인자는 `runtime_root_dir`, `runtime_bundle_dir`, `real_mode`,
`calibration_required`, `use_sim_time`, `rviz`다.

Guard는 release schema/hash, PGM/YAML, source DB와 checked-in INI를 검증한 뒤 mutable
runtime copy만 만든다. RTAB-Map은 `Mem/IncrementalMemory=false`로 그 copy를 읽는다.
Source raw DB와 release map에 직접 write하지 않는다. Hash/schema mismatch, 잘못된
capture 조합, symlink/hard-link, course 내부 runtime target은 정상적인 fail-closed
오류다. 원본을 고치지 말고 올바른 release 조합과 새 runtime bundle을 선택한다.

## 8. TF, slope, and live-layer interpretation

```text
map --RTAB-Map--> odom --local EKF--> base_link
                                      |-- lidar_link
                                      |-- imu_link
                                      |-- gps_link
                                      `-- camera_link
```

Wheel odometry는 `/slam/odometry/wheel`만 publish하며 TF owner가 아니다. Static
extrinsic은 한 supplier가 담당한다. Replay bag의 dynamic `/tf`를 다시 publish하면 이
ownership을 깨므로 기본 allowlist에서 제외했다.

Local EKF는 `two_d_mode=true`이고 wheel forward velocity와 IMU yaw/yaw-rate만 융합한다.
RTK는 local EKF 입력이 아니라 gate를 거쳐 RTAB-Map의 global observation으로 들어간다.
2D occupancy map은 고도를 담지 않는다. Scan leveler는 measured
`mount_xyz_rpy`, IMU roll/pitch를 사용하고 yaw를 제거하며, 현재 시작 제한은 tilt
30°, z -0.50~0.50 m다. 이 수치는 오르막·내리막 양 방향 field trial로 반드시
재결정한다.

## 9. What the synthetic smoke does not prove

다음 성공 명령은 hardware-free wiring과 안전 경계만 검증한다.

```bash
timeout --signal=INT --kill-after=5s 40s \
  rostest stier_slam_bringup synthetic_pipeline_smoke.test
```

테스트 전용 identity `map -> odom` fixture가 있고 `/rtabmap`은 의도적으로 없다.
따라서 다음의 증거로 사용할 수 없다.

- RTAB-Map loop closure 또는 실제 `map -> odom` 추정
- Mapping DB write/flush/reload와 frozen-DB localization
- Map/localization position accuracy 또는 GPS blackout drift
- 실제 센서 timestamp, extrinsic, wheel/steering calibration
- 오르막·내리막 성능, CPU/latency margin, 대회 코스 완주 안전성

이 항목들은 실제 dependency 설치 후 rosbag과 체크리스트의 offline/field acceptance로
별도 검증한다.
