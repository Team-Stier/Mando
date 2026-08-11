# RTAB-Map 2D SLAM Foundation Design

## 1. Purpose

Build a ROS1 Noetic foundation for a 1/5-scale autonomous vehicle that can:

1. record raw camera, 2D LiDAR, IMU, RTK-GNSS, wheel-speed, and steering data;
2. create a repeatable 2D occupancy map from manually driven rosbag recordings;
3. localize in the released map with RTAB-Map, IMU, wheel odometry, and RTK-GNSS;
4. edit static objects and semantic road annotations immediately before a run;
5. display temporary obstacles during a run without corrupting the released map.

The first deliverable is hardware-independent. Unknown USB sensor models are isolated behind adapters that publish stable ROS message contracts. Hardware driver selection and physical calibration happen after the exact models are known.

## 2. Scope

### Included

- Ubuntu 20.04, ROS1 Noetic, Catkin, and system Python 3.8
- Sensor-independent input contracts and health checks
- Ackermann wheel odometry from normalized vehicle speed and steering angle
- Local wheel/IMU fusion with `robot_localization`
- Optional roll/pitch scan leveling for sloped roads
- RTAB-Map 2D mapping and localization launch modes
- RTK `FIX` gating and GPS-loss behavior
- Immutable raw maps, editable overlays, semantic annotations, and versioned releases
- Browser-based pre-run map editing served only on localhost
- Temporary LiDAR obstacle layer with expiry
- rosbag recording profiles, synthetic data, unit tests, and launch smoke tests

### Excluded

- Global/local path planning
- Obstacle avoidance decisions
- Steering, throttle, and brake control
- Automatic camera lane or stop-line detection
- 3D elevation-map generation
- Vendor-specific USB drivers before hardware models are known
- A claim of real-vehicle accuracy before calibration and field tests

## 3. Acceptance Criteria

The hardware-independent release is accepted when:

- the Catkin workspace builds with `/usr/bin/python3`;
- all unit and launch tests pass;
- synthetic sensors can drive mapping and localization launch graphs without duplicate TF publishers;
- map overlay edits can add and erase occupancy, add semantic features, produce a new immutable release, and preserve the raw map;
- temporary obstacle cells appear and expire according to configured timeouts;
- stale, malformed, non-finite, out-of-order, or missing sensor data is rejected and reported;
- generated manifests contain hashes for every released artifact;
- the documented rosbag workflow can replay using `/clock` and deterministic timestamps.

Field acceptance targets, to be measured rather than assumed, are:

- position error at or below 0.10 m;
- heading error at or below 3 degrees;
- localization output at or above 10 Hz;
- continued local odometry during temporary RTK loss;
- no persistent duplicate curb or wall wider than 0.10 m after loop closure;
- temporary obstacles visible within 0.5 s and removed after their configured expiry.

## 4. System Architecture

```text
vendor drivers
    |
    v
sensor adapters ---> raw rosbag recorder
    |
    +--> /slam/input/scan_raw ----> scan_leveler ----------+
    +--> /slam/input/imu ----------------------------------+---> RTAB-Map 2D
    +--> /slam/input/gnss/fix -----------------------------+       |
    +--> speed + steering --> vehicle_odometry --> local_ekf       +--> map -> odom
                                                   |               +--> raw map database
                                                   +--> odom -> base_link

raw map database + occupancy image
    |
    v
map version manager <---- localhost map editor
    |
    +--> released occupancy map
    +--> static overlay
    +--> semantic layer

released map + leveled LiDAR + map pose
    |
    v
temporary obstacle layer --> visualization outputs with expiry
```

### Component boundaries

1. `stier_slam_core`
   - Pure geometry, validation, map versioning, overlay rasterization, and obstacle-expiry libraries.
   - ROS nodes for vehicle odometry, scan leveling, RTK gating, health monitoring, map release, and temporary obstacles.

2. `stier_slam_bringup`
   - Launch files for recording, mapping, localization, map serving, editing, and synthetic demonstration.
   - YAML configuration for frames, topics, EKF, RTAB-Map, timeouts, and map metadata.

3. `stier_slam_test_support`
   - Deterministic synthetic publishers and test maps.
   - It is never launched in a real vehicle profile.

Third-party RTAB-Map, `robot_localization`, `map_server`, and RViz remain external dependencies. Project code configures and supervises them; it does not fork their algorithms.

## 5. ROS Contracts

### Normalized inputs

| Topic | Type | Required content |
|---|---|---|
| `/slam/input/scan_raw` | `sensor_msgs/LaserScan` | finite timestamp, non-empty `frame_id`, valid angle and range metadata |
| `/slam/input/imu` | `sensor_msgs/Imu` | normalized orientation quaternion or explicit unavailable covariance; angular velocity in rad/s |
| `/slam/input/gnss/fix` | `sensor_msgs/NavSatFix` | valid status, finite latitude/longitude, meaningful covariance |
| `/slam/input/gnss/rtk_status` | `diagnostic_msgs/DiagnosticStatus` | `FIX`, `FLOAT`, or `NO_FIX` state supplied by the vendor adapter |
| `/slam/input/vehicle_speed` | `geometry_msgs/TwistWithCovarianceStamped` | forward speed in m/s and covariance |
| `/slam/input/steering_angle` | `std_msgs/Float64` | road-wheel steering angle in radians |
| `/slam/input/camera/front/image_raw` | `sensor_msgs/Image` | recorded in phase one; not consumed by SLAM |

### Core outputs

| Topic or TF | Type | Owner |
|---|---|---|
| `/slam/odometry/wheel` | `nav_msgs/Odometry` | `vehicle_odometry_node` |
| `/slam/odometry/local` | `nav_msgs/Odometry` | `robot_localization` local EKF |
| `odom -> base_link` | TF | local EKF only |
| `/slam/scan/leveled_points` | `sensor_msgs/PointCloud2` | `scan_leveler_node` |
| `/slam/gnss/fix_accepted` | `sensor_msgs/NavSatFix` | `rtk_gate_node` |
| `map -> odom` | TF | RTAB-Map only |
| `/slam/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | RTAB-Map remap |
| `/slam/map/released` | `nav_msgs/OccupancyGrid` | `map_server` remap |
| `/slam/live_obstacles/points` | `sensor_msgs/PointCloud2` | temporary obstacle node |
| `/slam/live_obstacles/grid` | `nav_msgs/OccupancyGrid` | temporary obstacle node |
| `/slam/diagnostics` | `diagnostic_msgs/DiagnosticArray` | project diagnostic nodes |

All adapters may remap vendor topic names, but the normalized names and units remain stable.

## 6. Frames and TF Ownership

```text
map -> odom -> base_link
                  +--> lidar_link
                  +--> imu_link
                  +--> gps_link
                  +--> camera_link
```

- RTAB-Map is the only `map -> odom` publisher.
- The local EKF is the only `odom -> base_link` publisher.
- `robot_state_publisher` or `static_transform_publisher` owns physical sensor extrinsics.
- Vendor drivers must not publish competing transforms after integration.
- Frame names contain no leading slash.

The 2D occupancy map does not encode road height. IMU roll and pitch are used only to stabilize LiDAR points and diagnose excessive tilt. The localization pose exposed to navigation remains planar.

## 7. Runtime Modes

### Record

- Start normalized adapters and health checks.
- Record raw and normalized sensor topics, TF, diagnostics, and `/clock` when simulated.
- Never run real suppliers and replay suppliers on the same normalized topic simultaneously.

### Map

- Replay a bag with `--clock` or drive manually at low speed.
- Local EKF provides continuous odometry.
- RTAB-Map uses leveled LiDAR, local odometry, IMU, and `/slam/gnss/fix_accepted`.
- Save the RTAB-Map database and exported base occupancy grid as immutable raw artifacts.

### Edit and release

- Open a localhost-only browser editor.
- Draw occupied polygons, free-space polygons, and semantic line or point features.
- Validate geometry, bounds, labels, and map metadata.
- Rasterize occupancy edits into a new release directory using atomic writes.
- Hash artifacts and update an active-release manifest only after validation succeeds.

### Localize

- Load a selected released map and the matching RTAB-Map database.
- Run RTAB-Map with incremental mapping disabled.
- Publish pose and TF only when required inputs and transforms are fresh.
- Keep the released map read-only.

### Live obstacle display

- Transform leveled LiDAR points into `map` at their measurement timestamp.
- Suppress points already explained by occupied released-map cells.
- Mark remaining points in a bounded local grid.
- Expire cells not observed within the configured time-to-live.
- Never write them into the released map automatically.

## 8. Map Artifact Format

```text
maps/<course_id>/
  raw/<capture_id>/
    map.db
    base.pgm
    base.yaml
    capture_manifest.json
  workspaces/<workspace_id>/
    source_release.json
    static_overlay.json
    semantic_layer.json
  releases/<release_id>/
    map.pgm
    map.yaml
    static_overlay.json
    semantic_layer.json
    manifest.json
  active_release.json
```

- Raw captures and releases are immutable.
- Overlay coordinates use map-frame meters, not image pixels.
- `static_overlay.json` contains ordered polygon operations with `occupied` or `free` values.
- `semantic_layer.json` contains typed points and polylines such as `lane_center`, `stop_line`, `intersection`, and `course_boundary`.
- `manifest.json` records schema version, source capture and immutable RTAB-Map database reference, map resolution and origin, creation time, and SHA-256 hashes.
- Activation uses an atomic JSON file replacement rather than a fragile mutable symlink.

## 9. RTK and Sensor Failure Policy

- Only RTK `FIX` measurements with finite coordinates, acceptable covariance, monotonic timestamps, and plausible innovation enter the graph as strong global observations.
- `FLOAT` is retained for diagnostics and may be configured as a weak observation after field characterization; the default is reject.
- `NO_FIX` is rejected.
- RTK loss does not stop local EKF odometry or LiDAR localization.
- RTK recovery is innovation-gated to prevent a sudden map jump.
- Stale IMU prevents leveled-scan publication when leveling is required.
- Invalid scan metadata, NaN ranges, missing TF, or time reversal clears affected buffers and emits an error diagnostic.
- Localization covariance above configured limits marks localization unhealthy; it does not fabricate a valid pose.
- Map hash or schema mismatch prevents release activation.
- Temporary-obstacle time reversal clears the expiry cache, supporting rosbag loops safely.

## 10. Configuration

Hardware-dependent values live in YAML and are not compiled into nodes:

- wheelbase, steering ratio, speed scale, encoder scale;
- LiDAR and IMU frame names and mounting transforms;
- sensor rates, maximum age, and timeout values;
- RTK covariance and innovation gates;
- scan range, tilt, height, and outlier filters;
- map resolution and tile/local-grid dimensions;
- live obstacle occupancy threshold and expiry;
- RTAB-Map and EKF parameters.

The initial example profile contains inactive sample geometry and is clearly marked `calibration_required: true`. Real-vehicle launch refuses to run until that flag is explicitly cleared after measurement.

## 11. Testing Strategy

### Unit tests

- Ackermann integration, reverse motion, zero speed, timestamp reversal, and steering limits
- Quaternion validation and roll/pitch leveling geometry
- Laser range filtering and non-finite input handling
- RTK status, covariance, timestamp, and innovation gating
- Overlay schema validation, polygon rasterization, free/occupied precedence, and bounds
- Immutable release creation, hashing, atomic activation, and corrupted-artifact rejection
- Temporary obstacle classification, expiry, and bag-time reversal

### ROS integration tests

- One owner for each TF edge
- Synthetic sensor topics satisfy normalized contracts
- Mapping launch consumes expected topics
- Localization launch disables incremental mapping
- Released map loads through `map_server`
- Diagnostics change state for stale and recovered inputs

### Offline and field tests

- Record at least two low-speed laps in both useful directions where permitted.
- Replay bags deterministically with system Python and `/use_sim_time`.
- Compare repeated wall and curb alignment before selecting a release.
- Measure RTK fixed/float/no-fix segments, local drift, loop-closure error, pose rate, CPU load, and latency.
- Test uphill, downhill, slope transitions, open feature-poor areas, and temporary obstacles separately.

## 12. Delivery Order

1. Catkin repository, package manifests, dependency checks, and topic/frame contracts.
2. Pure tested libraries for vehicle motion, sensor validation, RTK gating, map overlays, versioning, and obstacle expiry.
3. ROS nodes and synthetic publishers around those libraries.
4. EKF and RTAB-Map mapping/localization launch profiles.
5. Map editor and release workflow.
6. Full build, tests, synthetic end-to-end demonstration, and operator documentation.
7. Hardware adapter, calibration, rosbag, and course-tuning follow-up when models and measurements are available.

## 13. Environment Constraint

The interactive shell currently resolves `python3` to Conda Python 3.13, while ROS Noetic uses Ubuntu system Python 3.8. All Catkin configure, tests, and ROS Python entrypoints therefore use `/usr/bin/python3`; no global shell or Conda configuration is modified.
