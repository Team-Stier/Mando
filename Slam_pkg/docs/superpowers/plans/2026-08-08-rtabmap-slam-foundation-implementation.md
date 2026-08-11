# RTAB-Map SLAM Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a hardware-independent ROS1 Noetic workspace that records normalized sensors, creates and reloads RTAB-Map 2D maps, versions editable occupancy and semantic layers, and publishes expiring live obstacles.

**Architecture:** Pure Python libraries own deterministic math, validation, map artifacts, and expiry state; thin ROS nodes translate messages and own no reusable policy. `robot_localization` owns `odom -> base_link`, RTAB-Map owns `map -> odom`, and vendor drivers are replaceable adapters outside the core.

**Tech Stack:** Ubuntu 20.04, ROS1 Noetic, Catkin, `/usr/bin/python3` 3.8, `rospy`, RTAB-Map ROS, `robot_localization`, `map_server`, `tf2_ros`, standard-library HTTP server, HTML5 Canvas, `unittest`, `rostest`.

## Global Constraints

- Work only on `feature/rtabmap-slam-foundation`; do not create or modify `main`.
- Configure Catkin with `-DPYTHON_EXECUTABLE=/usr/bin/python3` because interactive `python3` resolves to Conda 3.13.
- Raw captures and released maps are immutable; activation uses atomic JSON replacement.
- Real mode refuses to start while `calibration_required: true`.
- RTK `FLOAT` and `NO_FIX` are rejected by default.
- Live obstacles never mutate a raw or released map.
- Camera data is recorded but is not a phase-one SLAM input.
- Unknown vendor drivers may only remap into the normalized contracts documented in the design.

## Locked File Structure

```text
CMakeLists.txt                         Catkin workspace top level
.gitignore                            Generated artifacts and local maps
README.md                             Operator workflows and limitations
scripts/bootstrap_dependencies.sh     Read-only check and explicit apt install modes
src/stier_slam_core/
  CMakeLists.txt                      Python package installation and tests
  package.xml                         Runtime and test dependencies
  setup.py                            Catkin Python setup
  src/stier_slam_core/
    __init__.py
    ackermann.py                      Vehicle integration only
    scan_leveling.py                  Quaternion and scan geometry only
    rtk_gate.py                       RTK policy only
    grid_map.py                       PGM/YAML and coordinate conversion only
    map_overlay.py                    Overlay schema and rasterization only
    map_release.py                    Immutable releases and hashes only
    live_obstacles.py                 Static suppression and expiry only
    freshness.py                      Timestamp/rate state only
  scripts/
    vehicle_odometry_node.py          ROS wrapper for AckermannIntegrator
    scan_leveler_node.py              LaserScan+IMU to PointCloud2 wrapper
    rtk_gate_node.py                  NavSatFix/status gate wrapper
    live_obstacle_node.py             TF, grid, and obstacle publishers
    sensor_health_node.py             DiagnosticArray publisher
    map_release_cli.py                 init/validate/release/activate commands
    map_editor_server.py              localhost editor API and static files
  web/
    index.html                        Editor controls and canvas
    app.js                            PGM rendering, draw/edit, API calls
    style.css                         Editor layout
  test/
    test_ackermann.py
    test_scan_leveling.py
    test_rtk_gate.py
    test_map_overlay.py
    test_map_release.py
    test_live_obstacles.py
    test_freshness.py
    test_editor_api.py
src/stier_slam_bringup/
  CMakeLists.txt
  package.xml
  config/vehicle.yaml
  config/ekf_local.yaml
  config/rtabmap_mapping.ini
  config/rtabmap_localization.ini
  config/timeouts.yaml
  launch/record.launch
  launch/mapping.launch
  launch/localization.launch
  launch/editor.launch
  launch/synthetic_demo.launch
  rviz/slam.rviz
  test/launch_contracts.test
  test/test_launch_contracts.py
src/stier_slam_test_support/
  CMakeLists.txt
  package.xml
  scripts/synthetic_sensor_node.py
  maps/demo/base.pgm
  maps/demo/base.yaml
  test/synthetic_topics.test
  test/test_synthetic_topics.py
```

---

### Task 1: Catkin Workspace and Contract Skeleton

**Files:**
- Create all top-level and package metadata files from the locked structure.
- Create: `src/stier_slam_core/src/stier_slam_core/__init__.py`
- Create: `src/stier_slam_core/test/test_imports.py`

**Interfaces:**
- Produces importable package `stier_slam_core` and Catkin packages `stier_slam_core`, `stier_slam_bringup`, and `stier_slam_test_support`.
- Declares ROS dependencies: `diagnostic_msgs`, `geometry_msgs`, `message_filters`, `nav_msgs`, `rospy`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `robot_localization`, `map_server`, `rtabmap_launch`, `rtabmap_slam`, `rosbag`, `rostest`.

- [ ] **Step 1: Write the failing import test**

```python
import unittest

class ImportTest(unittest.TestCase):
    def test_core_version_is_exposed(self):
        import stier_slam_core
        self.assertEqual(stier_slam_core.__version__, "0.1.0")
```

- [ ] **Step 2: Run the test and confirm the package does not exist**

Run: `/usr/bin/python3 -m unittest discover -s src/stier_slam_core/test -p 'test_imports.py' -v`

Expected: `ModuleNotFoundError: No module named 'stier_slam_core'`.

- [ ] **Step 3: Create the three Catkin packages and Python setup**

Use `catkin_python_setup()` in core, `catkin_install_python()` for every executable, `catkin_add_nosetests(test)`, and version `0.1.0` consistently in `package.xml`, `setup.py`, and `__init__.py`.

- [ ] **Step 4: Configure and build with system Python**

Run: `source /opt/ros/noetic/setup.bash && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`

Expected: all three packages configure and build; the import test passes through `PYTHONPATH=devel/lib/python3/dist-packages`.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt .gitignore README.md scripts src
git commit -m "build: scaffold ROS Noetic SLAM workspace"
```

### Task 2: Ackermann Wheel Odometry

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/ackermann.py`
- Create: `src/stier_slam_core/test/test_ackermann.py`
- Create: `src/stier_slam_core/scripts/vehicle_odometry_node.py`
- Create: `src/stier_slam_bringup/config/vehicle.yaml`

**Interfaces:**
- Produces `Pose2DState(x: float, y: float, yaw: float, linear_speed: float, yaw_rate: float, stamp: float)`.
- Produces `AckermannIntegrator(wheelbase_m, max_abs_steering_rad, max_abs_speed_mps).update(stamp, speed_mps, steering_rad) -> Pose2DState`.
- ROS node consumes `/slam/input/vehicle_speed` and `/slam/input/steering_angle`; publishes `/slam/odometry/wheel` without publishing TF.

- [ ] **Step 1: Write failing tests for straight, turn, reverse, and time reversal**

```python
state = AckermannIntegrator(0.32, 0.6, 8.0).update(1.0, 1.0, 0.0)
state = integrator.update(2.0, 1.0, 0.0)
self.assertAlmostEqual(state.x, 1.0)
with self.assertRaises(NonMonotonicTimeError):
    integrator.update(1.5, 1.0, 0.0)
```

Also assert `yaw_rate == speed * tan(steering) / wheelbase`, steering and speed limits reject rather than silently clamp, and reverse motion changes position with signed speed.

- [ ] **Step 2: Run the focused test and observe missing symbols**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_ackermann.py -v`

Expected: import failure for `stier_slam_core.ackermann`.

- [ ] **Step 3: Implement midpoint Ackermann integration**

Validate positive finite wheelbase, finite inputs, monotonic timestamps, and configured absolute limits. Use midpoint heading for translation and normalize yaw with `atan2(sin(yaw), cos(yaw))`.

- [ ] **Step 4: Implement the ROS wrapper and inactive calibration guard**

Require fresh steering before each speed update. Read `calibration_required`; exit with ROS fatal when `real_mode:=true` and it remains true. Fill pose/twist covariance from YAML and publish `nav_msgs/Odometry` in `odom` with child `base_link`.

- [ ] **Step 5: Run tests and commit**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_ackermann.py -v`

```bash
git add src/stier_slam_core src/stier_slam_bringup/config/vehicle.yaml
git commit -m "feat: add calibrated Ackermann wheel odometry"
```

### Task 3: Scan Validation and Slope Leveling

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/scan_leveling.py`
- Create: `src/stier_slam_core/test/test_scan_leveling.py`
- Create: `src/stier_slam_core/scripts/scan_leveler_node.py`

**Interfaces:**
- Produces `normalize_quaternion(qx, qy, qz, qw) -> tuple[float, float, float, float]`.
- Produces `level_scan(ranges, angle_min, angle_increment, range_min, range_max, quaternion, mount_xyz_rpy, min_z_m, max_z_m) -> list[tuple[float, float, float]]`.
- ROS node synchronizes `/slam/input/scan_raw` and `/slam/input/imu`; publishes `/slam/scan/leveled_points` in `base_link`.

- [ ] **Step 1: Write failing geometry and validation tests**

```python
points = level_scan([2.0], 0.0, 1.0, 0.1, 10.0,
                    quaternion_from_rpy(0.0, math.radians(10), 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), -1.0, 1.0)
self.assertAlmostEqual(points[0][0], 2.0 * math.cos(math.radians(10)), places=6)
```

Assert invalid quaternion, invalid metadata, all-NaN ranges, excessive tilt, and reversed range limits raise typed exceptions; isolated NaN/Inf ranges are skipped.

- [ ] **Step 2: Run and confirm the module is absent**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_scan_leveling.py -v`

- [ ] **Step 3: Implement dependency-free quaternion matrices and point projection**

Remove IMU yaw while retaining its roll/pitch leveling rotation, apply configured LiDAR-to-base mounting transform, filter by corrected z, and return finite XYZ points. Never interpolate invalid ranges into obstacles.

- [ ] **Step 4: Implement the synchronized ROS wrapper**

Use `ApproximateTimeSynchronizer(queue_size, slop)`, reject IMU orientation when `orientation_covariance[0] == -1`, enforce `max_abs_tilt_rad`, and publish `PointCloud2` stamped with the scan time. Emit throttled ROS errors on dropped frames.

- [ ] **Step 5: Run tests and commit**

```bash
git add src/stier_slam_core
git commit -m "feat: level 2D LiDAR scans with IMU attitude"
```

### Task 4: RTK FIX Gate

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/rtk_gate.py`
- Create: `src/stier_slam_core/test/test_rtk_gate.py`
- Create: `src/stier_slam_core/scripts/rtk_gate_node.py`

**Interfaces:**
- Produces `RtkFix(stamp, latitude, longitude, altitude, covariance_xy_m2, nav_status)`.
- Produces `RtkGate.evaluate(fix: RtkFix, rtk_state: str) -> GateDecision(accepted: bool, reason: str)`.
- ROS node consumes raw fix and RTK diagnostic status; republishes accepted messages on `/slam/gnss/fix_accepted` and diagnostics on `/slam/diagnostics/rtk_gate`.

- [ ] **Step 1: Write failing policy tests**

Cover `FIX`, `FLOAT`, `NO_FIX`, invalid NavSat status, latitude/longitude bounds, non-finite covariance, covariance above threshold, non-monotonic time, and innovation speed above `max_speed_mps + innovation_margin_m / dt`.

- [ ] **Step 2: Run and confirm import failure**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_rtk_gate.py -v`

- [ ] **Step 3: Implement deterministic gating**

Use an equirectangular local-distance approximation for successive fixes, update the previous accepted fix only on acceptance, and expose exact rejection reasons such as `rtk_not_fixed`, `covariance_too_large`, and `innovation_too_large`.

- [ ] **Step 4: Implement the ROS wrapper**

Cache RTK state with a maximum age, copy accepted `NavSatFix` without rewriting its timestamp, and publish status level `OK`, `WARN`, or `ERROR` with rejection counters.

- [ ] **Step 5: Run tests and commit**

```bash
git add src/stier_slam_core
git commit -m "feat: gate RTK fixes before graph optimization"
```

### Task 5: Map Grid, Overlay, and Immutable Releases

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/grid_map.py`
- Create: `src/stier_slam_core/src/stier_slam_core/map_overlay.py`
- Create: `src/stier_slam_core/src/stier_slam_core/map_release.py`
- Create: `src/stier_slam_core/test/test_map_overlay.py`
- Create: `src/stier_slam_core/test/test_map_release.py`
- Create: `src/stier_slam_core/scripts/map_release_cli.py`

**Interfaces:**
- `GridMap(width, height, resolution, origin_x, origin_y, cells)` uses ROS row-major cell order with values `-1`, `0`, and `100`.
- `load_ros_map(yaml_path) -> GridMap`; `write_ros_map(grid, pgm_path, yaml_path)` preserves standard map-server pixel semantics.
- `apply_overlay(grid, overlay_document) -> GridMap` consumes schema version 1 ordered polygons with operation `occupied` or `free` in map meters.
- `ReleaseManager.create_release(course_dir, capture_id, workspace_id, release_id, now_iso) -> pathlib.Path` is atomic and refuses overwrite.

- [ ] **Step 1: Write failing overlay tests**

Create a 10x10 grid at 0.1 m resolution. Assert map-to-cell boundary conversion, polygon occupancy, a later free polygon overriding an occupied polygon, malformed/self-too-small polygons rejected, and the source grid unchanged.

- [ ] **Step 2: Write failing release tests**

Assert release files include SHA-256 entries, repeated release IDs fail, a corrupted hash prevents activation, `../escape` IDs fail, active manifest replacement is atomic, and raw source bytes never change.

- [ ] **Step 3: Run focused tests and confirm missing modules**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_map_overlay.py src/stier_slam_core/test/test_map_release.py -v`

- [ ] **Step 4: Implement strict PGM/YAML, polygon, manifest, and atomic I/O**

Support P2 and P5 PGM input, write P5 output, reject unsupported `negate`, thresholds, truncated data, invalid origins, unknown schema fields, and non-finite coordinates. Create releases in a sibling temporary directory, fsync files, rename once, then verify hashes before activation.

- [ ] **Step 5: Implement CLI and run tests**

Commands are:

```text
map_release_cli.py init-capture --course DIR --capture ID --db FILE --map-yaml FILE
map_release_cli.py init-workspace --course DIR --capture ID --workspace ID
map_release_cli.py validate --course DIR --workspace ID
map_release_cli.py release --course DIR --workspace ID --release ID
map_release_cli.py activate --course DIR --release ID
```

- [ ] **Step 6: Commit**

```bash
git add src/stier_slam_core
git commit -m "feat: version editable occupancy map releases"
```

### Task 6: Temporary Obstacle Classification and Expiry

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/live_obstacles.py`
- Create: `src/stier_slam_core/test/test_live_obstacles.py`
- Create: `src/stier_slam_core/scripts/live_obstacle_node.py`

**Interfaces:**
- `LiveObstacleTracker(base_grid, ttl_sec, occupied_threshold).update(stamp, points_map) -> set[(int, int)]`.
- `snapshot(stamp) -> set[(int, int)]`; time reversal clears the tracker and records reason `time_reversal`.
- ROS node consumes released map and leveled cloud, transforms at measurement time, and publishes point cloud plus a bounded local occupancy grid.

- [ ] **Step 1: Write failing classification tests**

Assert points on occupied or unknown base cells are suppressed, points on free cells are tracked, duplicate cells coalesce, expiry is exact at `last_seen + ttl`, off-map points are ignored, and time reversal clears all cells.

- [ ] **Step 2: Run focused tests and confirm import failure**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_live_obstacles.py -v`

- [ ] **Step 3: Implement pure tracking state**

Store only map-cell keys and last-seen stamps. Bound input points and cache size. Treat unknown space as not classifiable by default to avoid inventing dynamic obstacles outside the mapped course.

- [ ] **Step 4: Implement the ROS wrapper**

Use `tf2_ros.Buffer.lookup_transform(map, cloud_frame, cloud_stamp)`, transform finite XYZ points, and publish only after a released map and transform are available. Local grid size, resolution, and TTL come from YAML; full-map grids are forbidden.

- [ ] **Step 5: Run tests and commit**

```bash
git add src/stier_slam_core
git commit -m "feat: publish expiring live obstacle layer"
```

### Task 7: Sensor Freshness and Diagnostics

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/freshness.py`
- Create: `src/stier_slam_core/test/test_freshness.py`
- Create: `src/stier_slam_core/scripts/sensor_health_node.py`
- Create: `src/stier_slam_bringup/config/timeouts.yaml`

**Interfaces:**
- `FreshnessMonitor.update(name, header_stamp, receipt_time) -> None`.
- `FreshnessMonitor.evaluate(now) -> dict[str, HealthState]`, where state is `OK`, `STALE`, `MISSING`, `FUTURE`, or `TIME_REVERSED`.
- Node publishes one `DiagnosticStatus` per normalized sensor plus aggregate state.

- [ ] **Step 1: Write failing state-transition tests**

Test missing before first sample, OK, stale at exact threshold, future stamp beyond tolerance, header reversal, bag-clock reversal reset, and recovery after a new monotonic sample.

- [ ] **Step 2: Run and confirm import failure**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_freshness.py -v`

- [ ] **Step 3: Implement monitor and ROS wrapper**

Keep ROS receipt time and message header time separate. Use ROS time for replay-compatible age, never replace source timestamps, and include last age/rate/reason as diagnostic key-values.

- [ ] **Step 4: Run tests and commit**

```bash
git add src/stier_slam_core src/stier_slam_bringup/config/timeouts.yaml
git commit -m "feat: monitor normalized sensor freshness"
```

### Task 8: RTAB-Map, EKF, Recording, and Localization Bringup

**Files:**
- Create or modify all `stier_slam_bringup/config`, `launch`, and `rviz` files in the locked structure.
- Create: `src/stier_slam_bringup/test/launch_contracts.test`
- Create: `src/stier_slam_bringup/test/test_launch_contracts.py`

**Interfaces:**
- `record.launch` records exact normalized topics and TF without starting suppliers.
- `mapping.launch` starts core preprocessing, local EKF, RTAB-Map incremental mapping, and diagnostics.
- `localization.launch` starts `map_server`, core preprocessing, local EKF, RTAB-Map with `Mem/IncrementalMemory=false`, live obstacles, and diagnostics.
- Launch arguments include `real_mode`, `database_path`, `map_yaml`, `use_sim_time`, and `rviz`.

- [ ] **Step 1: Write failing XML/contract tests**

Parse launch XML and assert exactly one node can publish each TF edge, normalized topic remaps exist, localization uses the localization INI, mapping uses the mapping INI, real mode carries the calibration guard, and no launch contains a vendor device path.

- [ ] **Step 2: Run the contract rostest and observe missing launch files**

Run: `source devel/setup.bash && rostest stier_slam_bringup launch_contracts.test`

- [ ] **Step 3: Add local EKF configuration**

Fuse wheel-odometry forward velocity and IMU yaw/angular velocity in `two_d_mode: true`; publish `odom -> base_link`; reject differential GPS input because RTK goes to RTAB-Map, not the local EKF.

- [ ] **Step 4: Add RTAB-Map configurations and launches**

Use ICP with `Reg/Force3DoF=true`, scan cloud input marked 2D, local odometry, IMU, and accepted GPS. Mapping is incremental; localization is read-only. RTAB-Map alone publishes `map -> odom`.

- [ ] **Step 5: Add recording and RViz profiles**

Record raw and normalized topics, `/tf`, `/tf_static`, `/diagnostics`, and `/clock`; exclude map release directories from automatic overwrite. RViz displays released map, leveled cloud, pose, TF, and live obstacles.

- [ ] **Step 6: Run launch checks and commit**

Run: `source devel/setup.bash && find src/stier_slam_bringup/launch -name '*.launch' -print0 | xargs -0 -n1 roslaunch-check`

```bash
git add src/stier_slam_bringup
git commit -m "feat: add RTAB-Map mapping and localization bringup"
```

### Task 9: Localhost Map Editor

**Files:**
- Create: `src/stier_slam_core/scripts/map_editor_server.py`
- Create: `src/stier_slam_core/web/index.html`
- Create: `src/stier_slam_core/web/app.js`
- Create: `src/stier_slam_core/web/style.css`
- Create: `src/stier_slam_core/test/test_editor_api.py`
- Create: `src/stier_slam_bringup/launch/editor.launch`

**Interfaces:**
- Bind only to `127.0.0.1` by default.
- `GET /api/workspace` returns map metadata, static overlay, and semantic layer.
- `GET /api/base.pgm` returns the source PGM bytes.
- `PUT /api/workspace` validates and atomically saves both JSON layers.
- `POST /api/release` validates, releases, and optionally activates a named release.

- [ ] **Step 1: Write failing HTTP API tests**

Start the server on an ephemeral localhost port with a temporary course. Assert non-local bind is rejected unless an explicit unsafe flag is passed, GET returns schema 1, valid PUT round-trips, invalid/path-traversal JSON returns 400, request bodies above 2 MiB return 413, and release returns artifact hashes.

- [ ] **Step 2: Run and confirm server module is absent**

Run: `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest src/stier_slam_core/test/test_editor_api.py -v`

- [ ] **Step 3: Implement the HTTP server using pure standard library**

Share all validation and release logic with `map_overlay.py` and `map_release.py`. Set `Cache-Control: no-store` for APIs, disallow directory listing, normalize all paths below the configured course root, and write JSON atomically.

- [ ] **Step 4: Implement Canvas editing**

Parse P2/P5 PGM in JavaScript, convert canvas pixels to map-frame meters using YAML metadata, support pan/zoom, occupied polygon, free polygon, semantic point/polyline, undo, save, validate, release, and activate. Display release ID and validation errors prominently.

- [ ] **Step 5: Run API tests and commit**

```bash
git add src/stier_slam_core src/stier_slam_bringup/launch/editor.launch
git commit -m "feat: add localhost pre-run map editor"
```

### Task 10: Deterministic Synthetic Demonstration

**Files:**
- Create all files in `src/stier_slam_test_support` from the locked structure.
- Create: `src/stier_slam_bringup/launch/synthetic_demo.launch`

**Interfaces:**
- Synthetic node publishes identity IMU, `FIX` RTK diagnostics/fixes, a bounded LaserScan, speed, steering, and static sensor transforms using a deterministic ROS-time sequence.
- `synthetic_demo.launch` defaults to `real_mode:=false` and never opens hardware devices.

- [ ] **Step 1: Write failing topic and timestamp rostest**

Wait conditionally for each normalized topic, assert types and non-empty frames, collect ten samples, assert monotonic stamps and expected minimum rate, and assert no `/dev/*` path is accessed by the test launch.

- [ ] **Step 2: Run and observe missing publisher**

Run: `source devel/setup.bash && rostest stier_slam_test_support synthetic_topics.test`

- [ ] **Step 3: Implement synthetic course sensors and demo map**

Publish a 360-degree scan whose ranges describe a rectangular course with a temporary obstacle that appears for a fixed interval. Publish speed and steering that form a repeatable loop; do not add random noise unless seeded and configured.

- [ ] **Step 4: Add demo launch and run rostests**

Run: `source devel/setup.bash && rostest stier_slam_test_support synthetic_topics.test && rostest stier_slam_bringup launch_contracts.test`

- [ ] **Step 5: Commit**

```bash
git add src/stier_slam_test_support src/stier_slam_bringup/launch/synthetic_demo.launch
git commit -m "test: add deterministic SLAM sensor demonstration"
```

### Task 11: Dependency Bootstrap, Operator Documentation, and Full Verification

**Files:**
- Create: `scripts/bootstrap_dependencies.sh`
- Expand: `README.md`
- Create: `docs/hardware-calibration-checklist.md`
- Create: `docs/rosbag-mapping-workflow.md`

**Interfaces:**
- Bootstrap `--check` makes no changes and lists missing APT/ROS packages.
- Bootstrap `--install` installs only the enumerated Noetic dependencies after printing them.
- Documentation gives exact record, replay, map, edit/release, localization, and verification commands.

- [ ] **Step 1: Add a failing shell contract check**

Run `scripts/bootstrap_dependencies.sh --check` before its implementation and record the missing-file failure. Add a test that rejects unknown flags and verifies `--check` performs no `apt` invocation.

- [ ] **Step 2: Implement dependency checks**

Check `ros-noetic-rtabmap-ros`, `ros-noetic-robot-localization`, `ros-noetic-map-server`, `ros-noetic-tf2-sensor-msgs`, `python3-yaml`, and build/test tools. Keep the package list explicit and sorted.

- [ ] **Step 3: Write operator and calibration documentation**

Document measured wheelbase, steering sign/ratio, encoder/RPM scale, LiDAR/IMU/GNSS extrinsics, IMU orientation convention, RTK status mapping, timestamp source, sensor rates, slope trials, and acceptance measurements. State that synthetic success is not field validation.

- [ ] **Step 4: Run complete verification from a clean generated workspace state**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make run_tests -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_test_results --verbose
find src -path '*/launch/*.launch' -print0 | xargs -0 -n1 roslaunch-check
scripts/bootstrap_dependencies.sh --check
git diff --check
git status --short
```

Expected: build exit 0, all tests pass with zero failures, every launch parses, dependencies are present, diff check is empty, and only intentional source files are tracked.

- [ ] **Step 5: Run a bounded synthetic end-to-end smoke test**

Launch the synthetic demo, wait for `/slam/odometry/wheel`, `/slam/odometry/local`, accepted RTK, leveled points, diagnostics, and live-obstacle output, then terminate cleanly. Save exact observed rates and unavailable RTAB-Map field evidence in README.

- [ ] **Step 6: Commit**

```bash
git add README.md scripts docs src
git commit -m "docs: add SLAM operation and calibration workflow"
```
