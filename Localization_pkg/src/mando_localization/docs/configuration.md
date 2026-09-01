# 설정 가이드

## 공통 원칙

- YAML은 launch가 각 노드 private namespace에 로드합니다. RViz 패널용으로 `localization_interfaces.yaml`과 `visualization.yaml`을 각각 `/mando_localization/interfaces`, `/mando_localization/visualization`에도 로드합니다.
- C++ 노드가 소비하는 필수 키는 누락·타입 오류·NaN/Inf·잘못된 범위일 때 기본값으로 숨기지 않고 시작을 중단합니다.
- 모든 변경은 관련 노드를 재시작해야 적용됩니다. `dynamic_reconfigure`는 없습니다.
- 실제 ROS node name은 launch argument가 기준입니다. `localization_interfaces.yaml.nodes`는 기본 이름과 진단 계약입니다.
- 공개 토픽은 YAML에서 관리하지만 드라이버/EKF/AMCL 내부 토픽은 고정 계약입니다. 고정값을 바꾸면 어댑터, EKF 또는 child launch 연결이 끊깁니다.
- RViz display 토픽은 `rviz/mando_localization.rviz`에도 문자열로 저장됩니다. 공개 토픽을 바꾸면 `.rviz`도 함께 갱신합니다.
- 표의 범위는 현재 C++ 검증 또는 외부 드라이버 계약을 기준으로 합니다. `config/*.yaml`의 실제 값이 최종 기준입니다.

## `localization_interfaces.yaml`

### 노드 이름

| 키 | 기본값 | 사용처 |
|---|---|---|
| `nodes.imu_driver` | `xsens_mti_node` | 장치/driver diagnostics; launch에서 덮어씀 |
| `nodes.gps_driver` | `ublox_gps_node` | 장치/driver diagnostics; launch에서 덮어씀 |
| `nodes.interface_adapter` | `localization_interface_adapter` | 공개↔내부 relay |
| `nodes.imu_normalizer` | `imu_normalizer` | IMU 검증 |
| `nodes.encoder_adapter` | `encoder_to_twist_adapter` | Encoder→Twist |
| `nodes.local_ekf` | `imu_encoder_local_ekf` | Local EKF 및 TF owner |
| `nodes.gps_quality_gate` | `odometry_gps_fusion` | GPS 직접 투영·게이트 |
| `nodes.lidar_quality_gate` | `odometry_lidar_fusion` | scan/AMCL 게이트 |
| `nodes.map_server`, `nodes.amcl` | `localization_map_server`, `localization_amcl` | 선택 LiDAR 위치추정 |
| `nodes.global_ekf` | `odometry_gps_lidar_global_ekf` | Global EKF 및 TF owner |
| `nodes.status_manager` | `localization_status_manager` | Supervisor 내부 센서 상태 평가 구성요소 |
| `nodes.relocalization_coordinator` | `relocalization_coordinator` | Supervisor 내부 장기 단절 복구 구성요소 |
| `nodes.supervisor`, `nodes.output_gate` | `localization_supervisor`, `localization_output_gate` | 공개 상태 중재/최종 출력 안전 |
| `nodes.static_tf_publisher` | `localization_static_tf_publisher` | 정적 TF 및 owner 선언 검증 |
| `nodes.visualization`, `nodes.rviz` | `localization_visualization`, `mando_localization_rviz` | Path·Marker/RViz |

모든 이름은 비어 있지 않은 string이어야 합니다. `nodes.status_manager`와
`nodes.relocalization_coordinator`는 Supervisor 프로세스 내부의 논리 구성요소
식별자이며 기본 bringup의 `rosnode list`에 별도 노드로 나타나지 않습니다.
실제 프로세스 이름은 `supervisor_node_name`으로 바꿉니다. 그 밖의 실행 노드
이름은 해당 `*_node_name` launch 인자를 사용하고, Local/Global EKF 이름은 TF
YAML owner와 일치시킵니다.

### 공개·상태 토픽

| 키 | 기본값 | 메시지/역할 |
|---|---|---|
| `topics.encoder_state` | `/erp42_serial/feedback` | Arduino `erp42_msgs/SerialFeedBack` 입력 |
| `topics.encoder_twist` | `/molit/vehicle/twist` | 표준 Twist 출력 |
| `topics.imu_data` | `/molit/sensors/imu/data` | raw IMU |
| `topics.imu_normalized` | `/molit/localization/imu/normalized` | 검증 IMU |
| `topics.gps_fix` | `/molit/sensors/gps/fix` | `NavSatFix` |
| `topics.gps_navpvt` | `/molit/sensors/gps/navpvt` | 고정 driver NavPVT를 ShapeShifter로 type-preserving relay |
| `topics.lidar_scan` | `/molit/sensors/lidar/scan` | raw LaserScan |
| `topics.local_odometry` | `/molit/localization/local/odometry` | 공개 Local EKF |
| `topics.gps_map_pose` | `/molit/localization/gps/map_pose` | 승인 GPS pose |
| `topics.lidar_map_pose` | `/molit/localization/lidar/map_pose` | 승인 AMCL pose |
| `topics.global_odometry` | `/molit/localization/global/odometry` | 공개 Global raw Odometry 및 Output Gate 입력 |
| `topics.output_odometry` | `/molit/localization/odometry` | 최종 gated Odometry |
| `topics.path`, `topics.markers` | `/molit/localization/path`, `/molit/localization/markers` | RViz Path/MarkerArray |
| `topics.status`, `topics.state`, `topics.valid` | `/molit/localization/status`, `/molit/localization/state`, `/molit/localization/valid` | diagnostics/enum/승인 |
| `topics.gps_relocalizing` | `/mando_localization/internal/gps/relocalizing` | Coordinator가 gate·장기 복구를 합친 latched Bool |
| `topics.lidar_relocalizing` | `/mando_localization/internal/lidar/relocalizing` | LiDAR 게이트 latched Bool |
| `topics.initialpose` | `/initialpose` | RViz 수동 초기값; 어댑터가 AMCL 내부 토픽으로 relay |
| `topics.map` | `/map` | 어댑터가 내부 map_server OccupancyGrid를 latched relay |
| `topics.diagnostics` | `/diagnostics` | 표준 진단 계약 기록; 현재 Supervisor 출력은 `topics.status` |
| `topics.recovery_state` | `/molit/localization/recovery/state` | Coordinator 복구 단계 String |

토픽은 `/`로 시작하는 절대 ROS 이름이어야 합니다.
`topics.gps_relocalizing`과 `topics.lidar_relocalizing`은 YAML 구조상 `topics`에
기록되어 있지만 외부 소비용 위치 토픽이 아니라 내부 상태 신호이며 반드시 서로
달라야 합니다. `topics.diagnostics=/diagnostics`는 예약·계약 기록용이고 현재
Supervisor 진단 출력은 `topics.status`를 사용합니다.

### Supervisor 내부 토픽과 EKF 서비스

| 키 | 기본값 | 실제 소유권·역할 |
|---|---|---|
| `internal_topics.evaluated_status` | `/mando_localization/internal/status/evaluated` | Status Manager → Supervisor 진단 후보 |
| `internal_topics.evaluated_state` | `/mando_localization/internal/status/state` | Status Manager → Supervisor 상태 후보 |
| `internal_topics.evaluated_valid` | `/mando_localization/internal/status/valid` | Status Manager → Supervisor 유효성 후보 |
| `internal_topics.gps_candidate_pose` | `/mando_localization/internal/gps/candidate_pose` | GPS gate의 Local innovation 전 quality candidate |
| `internal_topics.gps_gate_pose` | `/mando_localization/internal/gps/gate_pose` | 정상·단기 gate 승인 pose |
| `internal_topics.gps_gate_relocalizing` | `/mando_localization/internal/gps/gate_relocalizing` | GPS gate 자체 복구 상태 |
| `internal_topics.gps_reanchor_pose` | `/mando_localization/internal/gps/reanchor_pose` | Coordinator → GPS gate command |
| `internal_topics.gps_reanchor_accepted` | `/mando_localization/internal/gps/reanchor_accepted` | GPS gate 실제 적용 ACK |
| `internal_topics.recovery_active` | `/mando_localization/internal/recovery/active` | Coordinator heartbeat → Supervisor |
| `services.local_ekf_set_pose` | `/mando_localization/internal/ekf/local_set_pose` | Local EKF 전용 reset service |
| `services.global_ekf_set_pose` | `/mando_localization/internal/ekf/global_set_pose` | Coordinator가 쓰는 Global EKF 전용 reset service |

Reanchor command와 ACK는 `mando_localization/GpsGateReanchor`의 명시적
`uint64 transaction_id`와 `PoseWithCovarianceStamped pose`를 사용합니다.
`gps_candidate_pose`는 Global EKF로 relay하지 않으며 Coordinator만 공개 GPS
승인 권한을 갖습니다. Local/Global 서비스는 `robot_localization`의 기본 전역
`/set_pose` 충돌을 막기 위해 서로 다른 이름이어야 합니다.

### frame·메타데이터

| 키 | 기본값/제약 |
|---|---|
| `schema_version` | 정수 `1` |
| `frames.map/odom/base_link` | `map`, `odom`, `base_link` |
| `frames.imu/gps/lidar` | `imu_link`, `gps_link`, `laser_link` |
| `message_types.*` | 현재 ROS 메시지 계약을 기록하는 string |
| `contracts.coordinate_convention` | `ENU` |
| `contracts.planar_motion` | `true` |
| `contracts.speed_unit/angular_velocity_unit/angle_unit` | `m/s`, `rad/s`, `rad` |
| `contracts.configuration_reload` | `node_restart_required` |

frame은 비어 있지 않고 선행 `/`가 없어야 합니다. `message_types`와
`contracts`를 바꾸는 것만으로 C++ 메시지 타입이나 계산이 바뀌지는 않습니다.

### launch에 고정된 기타 내부 토픽

| 내부 계약 | 값 |
|---|---|
| driver IMU/GPS/NavPVT | `/mando_localization/internal/driver/imu`, `/mando_localization/internal/driver/gps_fix`, `/mando_localization/internal/driver/gps_navpvt` |
| EKF IMU/Twist | `/mando_localization/internal/ekf/imu`, `/mando_localization/internal/ekf/twist` |
| EKF GPS/LiDAR pose | `/mando_localization/internal/ekf/gps_pose`, `/mando_localization/internal/ekf/lidar_pose` |
| EKF Local/Global output | `/mando_localization/internal/ekf/local_odometry`, `/mando_localization/internal/ekf/global_odometry` |
| AMCL map/scan/pose/initialpose | `/mando_localization/internal/amcl/map`, `/mando_localization/internal/amcl/scan`, `/mando_localization/internal/amcl/pose`, `/mando_localization/internal/amcl/initialpose` |

## `relocalization_policy.yaml`

| 키 | 기본값 | 의미 |
|---|---:|---|
| `relocalization.long_outage_sec` | `2.0 s` | 마지막 gate 승인 receipt 후 `>`일 때 장기 복구 시작 |
| `relocalization.candidate_max_message_age_sec` | `1.0 s` | quality candidate 과거 한계 |
| `relocalization.candidate_max_future_stamp_sec` | `0.10 s` | Coordinator의 candidate 및 LiDAR/Twist/Global freshness 미래 한계 |
| `relocalization.candidate_cluster_radius_m` | `2.0 m` | 군집 첫 GPS 후보 기준 반경 |
| `relocalization.reanchor_ack_timeout_sec` | `1.0 s` | GPS gate 적용 ACK 대기 한계 |
| `lidar_assisted.required_consecutive_candidates` | `3` | LiDAR 교차검증을 매번 통과한 증가 timestamp 후보 수 |
| `lidar_assisted.max_gps_lidar_distance_m` | `5.0 m` | 각 GPS↔LiDAR pair의 최대 XY 차이 |
| `lidar_assisted.max_timestamp_skew_sec` | `0.20 s` | 각 GPS↔LiDAR pair의 최대 시각 차이 |
| `gps_only.automatic_reset_enabled` | `false` | LiDAR 없는 자동 Global EKF reset 활성화 |
| `gps_only.require_measured_datum` | `true` | `reference.measured=false`이면 reset 차단; mode·source는 Coordinator가 재검증하지 않음 |
| `gps_only.required_consecutive_candidates` | `5` | 정지 중 증가 timestamp GPS 후보 수 |
| `gps_only.max_stationary_speed_mps` | `0.30 m/s` | reset을 허용하는 정지·저속 한계 |
| `gps_only.global_confirmation_timeout_sec` | `1.0 s` | reset 후 Global 결과 확인 제한 |
| `gps_only.max_global_confirmation_distance_m` | `2.0 m` | reset 목표와 Global 결과 허용 거리 |
| `gps_only.required_global_confirmations` | `3` | reset 이후 서로 다른 증가 timestamp 결과 수 |
| `gps_only.set_pose_service_wait_timeout_sec` | `0.30 s` | Global 전용 service 존재 대기 한계 |

LiDAR 보조 경로는 mismatch, 중복·역행 timestamp 또는 GPS-only/LiDAR 모드
전환에서 후보 군집을 초기화합니다. GPS-only 경로는 Twist가 없거나
`|vx| > 0.30 m/s`이면 이동 후보를 세지 않습니다. 어느 경로든
transaction·stamp·XY가 일치하는 gate 적용 ACK 전에는 GPS pose를 공개하지
않습니다. LiDAR pose가 Global EKF의 구성 입력이어서 코드상 `set_pose`를
호출하지 않지만, 해당 측정이 EKF 내부에서 수용됐다는 ACK는 확인하지 않습니다.
증가 timestamp 조건은 GPS candidate에만 적용되며 같은 fresh LiDAR pose를 여러
비교에 재사용할 수 있습니다. GPS-only 분기는 LiDAR 비활성·invalid·stale 또는
pair skew 초과에서도 선택될 수 있지만, fresh·time-aligned LiDAR와 5 m 넘게
불일치하면 우회하지 않습니다.

GPS-only reset 요청은 최신 Global yaw·Z·나머지 covariance를 복사하고 GPS x/y와
XY covariance를 교체합니다. post-reset 확인은 timestamp와 XY 거리만 검사합니다.
`require_measured_datum`도 `reference.measured` bool만 확인하므로 운영에서는
`manual_datum`, `measured_at`, `source`를 별도로 검증해야 합니다.
Coordinator의 fresh Twist·Global은 receipt/stamp age 0.20초 이하이고 미래 stamp
0.10초 이하입니다. 이는 Status Manager의 해당 소스 미래 한계 0.05초와 다른
복구 전용 검사입니다.
`set_pose_service_wait_timeout_sec`는 service 발견 대기값이며 ROS1 동기 service
call 자체의 hard timeout은 아닙니다. Global 확인 timeout에는 service 대기·호출
시간이 포함되고, 동기 call 동안 같은 프로세스의 Manager·Coordinator·Supervisor
callback과 heartbeat도 지연될 수 있습니다. Output Gate는 마지막 `valid`가
stale해지면 최종 Odometry를 차단합니다.

## `imu_driver.yaml`

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `port` | string | `/dev/imu` | Xsens `DB8GG04M`에 고정된 직렬 별칭 |
| `device_id` | string | `03889250` | 2026-08-30 auto-scan으로 확인한 MTi-3 ID |
| `baudrate` | int, baud | `115200` | 실장치 auto-scan 확인값 |
| `frame_id` | string | `imu_link` | IMU frame 계약 |
| `publisher_queue_size` | int | `10` | 양수 |
| `pub_imu` | bool | `true` | 표준 IMU 출력 |
| `pub_quaternion/mag/angular_velocity/acceleration/free_acceleration/dq/dv/sampletime/temperature/pressure/gnss/twist/positionLLA/velocity` | bool | 모두 `false` | 불필요한 별도 출력 억제 |
| `pub_transform` | bool | `false` | TF 중복 방지; false 유지 |
| `hardware.usb_serial` | string | `DB8GG04M` | udev 식별 기록 |
| `hardware.calibration_state` | string | `unverified` | 실차 검증 상태 기록 |
| `covariance_override.enabled` | bool | `true` | 아래 정지 실측 stddev로 covariance 생성 |
| `covariance_override.calibration_state` | enum | `measured` | `unmeasured/measured/verified`; 현재 운영 검증 전 |
| `covariance_override.measured_at` | string | `2026-08-31T01:02:55+09:00/599.717s` | 측정 시각·bag 길이 기록 |
| `covariance_override.source` | string | 정지·모터 OFF bag, 첫 60 s 제외 | 측정 원본·절차 |
| `covariance_override.linear_acceleration_stddev_mps2` | double[3], m/s² | `[0.003685438, 0.003944295, 0.004177912]` | 모두 `>0` |
| `covariance_override.angular_velocity_stddev_radps` | double[3], rad/s | `[0.000319255, 0.000331483, 0.000300778]` | 모두 `>0` |
| `covariance_override.orientation_stddev_rad` | double[3], rad | `[0.000294491, 0.000363537, 0.000477059]` | 모두 `>0` |
| `imu.max_message_age_sec` | double, s | `0.25` | `>0`; normalizer 과거 한계 |
| `imu.max_future_stamp_sec` | double, s | `0.05` | `>=0` |
| `imu.min_quaternion_norm` | double | `1e-12` | `>0` |
| `imu.max_quaternion_normalization_error` | double | `0.001` | `>0` |
| `imu.max_covariance_diagonal` | double, SI² | `1000000` | `>0` |
| `imu.require_positive_covariance_diagonal` | bool | `true` | override 비활성일 때 raw 세 covariance의 X/Y/Z 대각이 모두 `>0`이 아니면 차단 |

override는 Xsens 드라이버가 아니라 `ImuNormalizer`가 정규화 출력의 세 covariance 대각에 표준편차의 제곱을 채우는 기능입니다. 2026-08-30 실제 `MTi-3-8A7G6`는 약 100.95 Hz, `imu_link`, 정상 timestamp를 발행했지만 원본 covariance 배열은 모두 0이었습니다. 현재 값은 2026-08-31 정지·모터 OFF bag의 noise floor이므로 `measured`일 뿐 `verified`가 아니며, 실제 주행·진동·방향 기준 시험 뒤 교체해야 합니다.

## `gps_driver.yaml`

| 키 | 타입·단위 | 기본값 | 영향 |
|---|---|---|---|
| `device` | string | `/dev/mando_gps` | ZED-F9P UART1 별칭 |
| `frame_id` | string | `gps_link` | NavSatFix frame |
| `uart1.baudrate` | int, baud | `460800` | UART1 속도 |
| `uart1.in`, `uart1.out` | int bitmask | `1`, `1` | UBX 입·출력 |
| `config_on_startup` | bool | `true` | 시작 때 UART1 RAM 설정을 UBX로 전환 |
| `save_on_shutdown`, `clear_bbr` | bool | `false`, `false` | Flash/BBR 자동 변경 방지 |
| `rate` | double, Hz | `10.0` | 측정 주기 계약 |
| `nav_rate` | int | `1` | 항법 계산 분주 |
| `dynamic_model` | string | `automotive` | 차량 동역학 모델 |
| `fix_mode` | string | `both` | KumarRobotics u-blox 2D/3D 계약값 |
| `enable_ppp` | bool | `false` | PPP 비활성 |
| `debug` | int | `0` | 드라이버 로그 |
| `publish.all` | bool | `false` | 전체 메시지 억제 |
| `publish.nav.pvt`, `publish.nav.relposned` | bool | `true`, `false` | NavPVT 사용, RELPOSNED 미사용 |
| `publish.nav.posecef`, `publish.nav.posllh` | bool | `false`, `false` | 추가 메시지 억제 |
| `hardware.usb_serial/receiver/interface` | string | `DBT042C1`, `ZED-F9P`, `C099_UART1` | 장치 기록 |
| `hardware.rtk_correction_source` | string | `none` | NTRIP/RTCM 범위 밖 |

이 파일은 외부 `ublox_gps` 패키지용입니다. 2026-08-30 `/dev/ttyUSB1` 460800 baud에서 확인한 실제 출력은 NMEA `GNRMC/GNGGA`이고 상태는 `V/0` no-fix였습니다. 드라이버가 시작하면 UART1의 휘발성 설정을 UBX in/out으로 바꾸되 `save_on_shutdown=false`라 Flash에는 저장하지 않습니다. 현재 호스트에는 `ublox_gps`가 없어 실제 UBX 전환과 ROS `NavSatFix`/`NavPVT` rate·frame·timestamp·covariance는 미검증입니다.

## `encoder_calibration.yaml`

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `encoder.speed_scale` | double | `1.0` | finite, `>0`; `SerialFeedBack.speed` 배율 |
| `encoder.direction_sign` | int | `1` | `-1` 또는 `1` |
| `encoder.speed_variance_m2ps2` | double, (m/s)² | `0.25` | `>=0` |
| `encoder.unobserved_variance` | double, SI² | `1000000` | `>0` |
| `encoder.max_abs_speed_mps` | double, m/s | `30.0` | `>0` |
| `encoder.max_abs_encoder_delta_100ms` | int, tick | `100000` | `>0` |
| `encoder.require_alive_counter_change` | bool | `true` | 연속 메시지의 uint8 `alive` counter가 변해야 함(255→0 랩 허용) |
| `encoder.calibration_state` | enum | `unmeasured` | `unmeasured/measured/verified` |
| `encoder.encoder_delta_window_sec` | double, s | `0.10` | `>0` |
| `encoder.meter_per_tick_enabled` | bool | `false` | true일 때 delta 일관성 검사 활성 |
| `encoder.meter_per_tick_m` | double, m/tick | `0.0` | 비활성 시 0 허용; 활성 시 `>0` |
| `encoder.delta_consistency_tolerance_mps` | double, m/s | `1.0` | `>0` |
| `encoder.steering.enabled` | bool | `false` | 보정 전 false |
| `encoder.steering.adc_center` | int | `0` | ADC 중심 |
| `encoder.steering.adc_per_rad` | double, count/rad | `0.0` | 활성 시 finite, 0이 아니어야 함 |
| `encoder.steering.min_adc/max_adc` | int | `0`, `65535` | min < max |

현재 Twist 계산은 `SerialFeedBack.speed`만 사용합니다. `steer`는 범위·보정 진단이고 yaw rate를 만들지 않습니다. `brake`도 상태 표시용입니다. 원본 메시지에 Header가 없어 출력 Twist timestamp는 PC 수신 시각입니다.

## `tf_configuration.yaml`

| 키 | 타입·기본값 | 제약·영향 |
|---|---|---|
| `schema_version` | int `1` | 정확히 1 |
| `strict_validation` | bool `true` | 반드시 true |
| `base_link_reference.origin` | string `rear_axle_center` | `base_link` 물리 원점, 비어 있지 않아야 함 |
| `base_link_reference.x_axis/y_axis/z_axis` | string | `forward/left/up` 고정, REP-103 |
| `dynamic_transforms[].parent_frame/child_frame` | string | 기본 `map->odom`, `odom->base_link` |
| `dynamic_transforms[].owner_node` | string | launch의 Global/Local EKF 이름과 일치 |
| `dynamic_transforms[].source` | string | 비어 있지 않은 소유 근거 |
| `static_transforms[].parent_frame/child_frame` | string | 유효 frame, 서로 달라야 함 |
| `static_transforms[].translation_m` | double[3], m | 길이 3, finite |
| `static_transforms[].rotation_rpy_deg` | double[3], degree | 길이 3, finite |
| `static_transforms[].enabled` | bool | 발행 요청 |
| `static_transforms[].calibration_state` | enum | `unmeasured/measured/verified` |
| `static_transforms[].measured_at/source` | string | 측정 이력 |

정적 TF는 `enabled=true`이고 상태가 `measured` 또는 `verified`일 때만 발행합니다. static+dynamic 전체에서 중복 child와 cycle을 거부합니다. 자세한 절차는 [tf_frames.md](tf_frames.md)를 참고하십시오.

현재 부분 측정으로 IMU X는 `+0.25 m`, LiDAR X는 `+1.05 m`가 저장돼 있습니다. Y/Z/RPY는 아직 자리표시자 0이므로 두 TF 모두 `unmeasured`, `enabled=false`이며 발행되지 않습니다.

## `ekf_local.yaml`, `ekf_global.yaml`

두 파일은 `robot_localization/ekf_localization_node` 표준 파라미터입니다.

| 키 | Local / Global 기본값 | 의미 |
|---|---|---|
| `frequency` | `30.0 / 30.0` Hz | filter 주기 |
| `sensor_timeout` | `0.15 / 0.20` s | 입력 timeout |
| `two_d_mode` | 둘 다 `true` | 2D 상태 제한 |
| `transform_time_offset` | `0.0` s | TF 시간 오프셋 |
| `transform_timeout` | `0.05` s | TF 대기 |
| `print_diagnostics` | `true` | filter 진단 |
| `debug` | `false` | debug 로그 |
| `publish_tf` | `true` | 각자 유일 TF 발행 |
| `publish_acceleration` | `false` | acceleration 출력 안 함 |
| `reset_on_time_jump` | `true` | ROS time 역행 reset |
| `map_frame/odom_frame/base_link_frame` | `map/odom/base_link` | frame 계약 |
| `world_frame` | `odom / map` | Local/Global 기준 |
| `twist0` | 둘 다 `/mando_localization/internal/ekf/twist` | `vx`만 true |
| `imu0` | 둘 다 `/mando_localization/internal/ekf/imu` | `yaw`, `vyaw` true |
| `pose0` | Global만 `/mando_localization/internal/ekf/gps_pose` | `x`, `y` true |
| `pose1` | Global만 `/mando_localization/internal/ekf/lidar_pose` | `x`, `y`, `yaw` true |
| `*_queue_size` | 10 또는 IMU 20 | 입력 큐 |
| `*_nodelay` | `true` | 즉시 처리 |
| `*_differential`, `*_relative` | `false` | 절대/직접 관측 |
| `*_rejection_threshold` | `5.0` | robot_localization innovation gate |
| `imu0_remove_gravitational_acceleration` | `true` | IMU 중력 제거 |
| `use_control` | `false` | 제어 입력 미사용 |
| `dynamic_process_noise_covariance` | `false` | 고정 Q |
| `process_noise_covariance` | double[225] | 상태 순서가 주석에 기록된 15×15 Q |

내부 토픽 값을 바꾸면 인터페이스 어댑터와 연결이 끊기므로 차량별 설정에서 유지합니다. Global에 Local Odometry 입력을 추가하지 않습니다.

## `gps_reference.yaml`

이 패키지는 별도 NavSat 변환 설정 파일을 사용하지 않습니다. 다음 키는 custom GPS 게이트가 직접 소비합니다.

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `reference.mode` | enum | `first_fix` | `first_fix` 또는 `manual_datum` |
| `reference.measured` | bool | `false` | `manual_datum`이면 반드시 true |
| `reference.measured_at` | string | `""` | `manual_datum`이면 비어 있지 않은 측정 시각 필수 |
| `reference.source` | string | `""` | `manual_datum`이면 비어 있지 않은 측량 원본·절차 필수 |
| `reference.latitude_deg` | double, degree | `0.0` | manual에서 `[-90,90]` |
| `reference.longitude_deg` | double, degree | `0.0` | manual에서 `[-180,180]` |
| `reference.altitude_m` | double, m | `0.0` | manual에서 finite |
| `reference.yaw_offset_rad` | double, rad | `0.0` | finite; ENU→map 회전 |
| `reference.map_x_m/map_y_m/map_z_m` | double, m | 모두 `0.0` | manual datum의 지도 좌표 |
| `reference.require_measured_datum_for_lidar` | bool | `true` | true+미측정이면 GPS-assisted AMCL 초기화 비활성 |
| `quality.minimum_fix_status` | int | `0` | NavSatStatus FIX~GBAS 범위 |
| `quality.max_message_age_sec` | double, s | `1.0` | `>0` |
| `quality.max_future_stamp_sec` | double, s | `0.10` | `>=0` |
| `quality.fallback_horizontal_variance_m2` | double, m² | `25.0` | unknown covariance fallback, `>0` |
| `quality.fallback_vertical_variance_m2` | double, m² | `100.0` | `>0` |
| `quality.max_horizontal_variance_m2` | double, m² | `25.0` | `>0`, fallback 이상 |
| `quality.max_vertical_variance_m2` | double, m² | `100.0` | `>0`, fallback 이상 |
| `quality.max_step_distance_m` | double, m | `50.0` | `>0`; 후보 연속성 |
| `quality.unobserved_variance` | double, SI² | `1000000` | `>0`; 미관측 pose 축 |
| `quality.max_position_innovation_m` | double, m | `10.0` | `>0`; Local Odom 예측 대비 |
| `quality.max_mahalanobis_distance` | double | `5.0` | `>0`; 결합 covariance gate |
| `quality.max_reanchor_candidate_distance_m` | double, m | `2.0` | `>0`; reanchor 명령과 최신 quality candidate 허용 거리 |

`first_fix` 모드에서는 YAML의 datum 좌표와 `map_x/y/z_m`을 사용하지 않고 최초 유효 fix를 `(0,0,0)`으로 설정합니다. 실지도는 반드시 측량 `manual_datum`을 사용합니다. 연속 횟수와 Local Odom/GPS timeout은 `status_policy.yaml`에서 공유합니다.

## `lidar_localization.yaml`

### AMCL

| 키 그룹 | 기본값 | 제약·영향 |
|---|---|---|
| `use_map_topic` | `true` | AMCL이 launch에서 remap된 고정 내부 map 토픽을 구독 |
| `tf_broadcast` | `false` | launch도 false 강제 |
| `global_frame_id/odom_frame_id/base_frame_id` | `map/odom/base_link` | frame |
| `transform_tolerance` | `0.20` s | TF 허용 |
| `gui_publish_rate`, `save_pose_rate` | `5.0`, `0.5` Hz | AMCL 출력/저장 |
| `odom_model_type` | `diff` | motion model |
| `odom_alpha1..4`, `odom_alpha5` | `0.2`, `0.1` | odom noise |
| `laser_model_type` | `likelihood_field` | sensor model |
| `laser_max_beams` | `60` | beam 수 |
| `laser_min_range/max_range` | `0.05/30.0` m | 사용 범위 |
| `laser_z_hit/short/max/rand` | `0.80/0.05/0.05/0.10` | mixture weight |
| `laser_sigma_hit`, `laser_lambda_short` | `0.20`, `0.10` | sensor model |
| `laser_likelihood_max_dist` | `2.0` m | likelihood field 한계 |
| `min_particles/max_particles` | `500/3000` | particle 범위 |
| `kld_err/kld_z` | `0.05/0.99` | KLD sampling |
| `update_min_d/update_min_a` | `0.10` m/rad | update 이동량 |
| `resample_interval` | `1` | resampling |
| `recovery_alpha_slow/fast` | `0.0/0.0` | AMCL random recovery 비활성 |

### LiDAR 게이트

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `scan_validation.timeout_sec` | double, s | `0.30` | `>0` |
| `scan_validation.min_range_count` | int | `20` | `>0` |
| `scan_validation.min_finite_ratio` | double | `0.50` | `[0,1]` |
| `scan_validation.require_laser_frame` | bool | `true` | `laser_link` 강제 |
| `quality.pose_max_message_age_sec` | double, s | `0.50` | `>0` |
| `quality.max_future_stamp_sec` | double, s | `0.10` | `>=0` |
| `quality.max_position_variance_m2` | double, m² | `4.0` | `>0` |
| `quality.max_yaw_variance_rad2` | double, rad² | `0.2741556778` | `>0` |
| `quality.max_pose_jump_m` | double, m | `3.0` | `>0` |
| `quality.max_yaw_jump_rad` | double, rad | `0.7853981634` | `>0` |
| `quality.stable_candidate_radius_m` | double, m | `1.0` | `>0` |
| `quality.stable_candidate_yaw_rad` | double, rad | `0.35` | `>0` |
| `quality.min_quaternion_norm` | double | `1e-12` | `>0` |
| `quality.max_quaternion_normalization_error` | double | `0.001` | `>0` |
| `initialization.use_gps_initialpose` | bool | `true` | 최초 GPS-assisted `/initialpose` |
| `initialization.gps_initial_yaw_rad` | double, rad | `0.0` | finite; heading 자리표시자 |
| `initialization.gps_initial_yaw_variance_rad2` | double, rad² | `9.8696044011` | `>0`; 넓은 yaw 불확실성 |
| `initialization.gps_initial_yaw_calibration_state` | enum | `unmeasured` | `unmeasured/measured/verified` |

실제 map 파일은 YAML이 아니라 launch `map_file` 인자로 지정합니다. AMCL map/scan/pose/initialpose 네 토픽은 C++와 child launch의 고정 내부 계약이므로 LiDAR YAML에 키가 없습니다.

## `status_policy.yaml`

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `manager_rate_hz` | double, Hz | `20.0` | `>0` |
| `startup_grace_sec` | double, s | `3.0` | `>=0` |
| `devices.imu/gps` | string | `/dev/imu`, `/dev/mando_gps` | 장치 diagnostics |
| `quality.max_position_variance_m2` | double, m² | `25.0` | `>0`; Manager/Output Gate |
| `sources.imu.timeout_sec/max_future_stamp_sec` | s | `0.20/0.05` | `>0` / `>=0` |
| `sources.encoder.timeout_sec/max_future_stamp_sec` | s | `0.20/0.05` | 원본과 Twist 공통 |
| `sources.local_odometry.timeout_sec/max_future_stamp_sec` | s | `0.20/0.05` | GPS 예측에도 timeout 사용 |
| `sources.global_odometry.timeout_sec/max_future_stamp_sec` | s | `0.20/0.05` | Global 출력 건강도 |
| `sources.gps.timeout_sec/max_future_stamp_sec` | s | `1.00/0.10` | gap와 상태 |
| `sources.lidar.timeout_sec/max_future_stamp_sec` | s | `0.50/0.10` | gap와 상태 |
| `absolute_sources.gps_enabled` | bool | `true` | launch가 `enable_gps_fusion`으로 덮어씀 |
| `absolute_sources.lidar_enabled` | bool | `false` | launch가 `enable_lidar_localization`으로 덮어씀 |
| `absolute_sources.recovery_consecutive_measurements` | int | `3` | `>0`; 두 gate 공유 |
| `absolute_sources.conflicting_sources_enter_relocalizing` | bool | `true` | source conflict 사용 |
| `absolute_sources.max_cross_source_distance_m` | double, m | `5.0` | `>0` |
| `absolute_sources.max_global_consistency_distance_m` | double, m | `10.0` | `>0` |
| `dead_reckoning.max_duration_sec` | double, s | `2.0` | `>0` |
| `dead_reckoning.max_distance_m` | double, m | `10.0` | `>0` |
| `dead_reckoning.limit_policy` | enum | `first_exceeded` | 이 값만 허용 |
| `output_gate.max_odometry_age_sec` | double, s | `0.20` | `>0` |
| `output_gate.max_valid_age_sec` | double, s | `0.50` | `>0` |
| `output_gate.max_future_stamp_sec` | double, s | `0.05` | `>=0` |
| `output_gate.max_quaternion_normalization_error` | double | `0.001` | `>0` |
| `output_gate.max_covariance_diagonal` | double, SI² | `1000000` | `>0` |
| `output_gate.require_finite_values` | bool | `true` | false면 시작 거부 |
| `output_gate.require_map_frame` | bool | `true` | false면 시작 거부 |
| `output_gate.publish_last_pose_when_invalid` | bool | `false` | true면 시작 거부 |
| `supervisor.max_evaluation_age_sec` | double, s | `0.50` | evaluated state/valid와 recovery heartbeat freshness; 초과 시 `FAULT` |

자세한 전이는 [status_and_recovery.md](status_and_recovery.md)를 참고하십시오.

## `visualization.yaml`

| 키 | 타입·단위 | 기본값 | 제약·영향 |
|---|---|---|---|
| `publish_rate_hz` | double, Hz | `10.0` | `>0`; MarkerArray timer |
| `fixed_frame` | string | `map` | 비어 있지 않음 |
| `path.max_pose_count` | int | `2000` | `>0` |
| `path.clear_on_invalid` | bool | `true` | valid 하강 시 Path 삭제 |
| `path.min_translation_m` | double, m | `0.05` | `>=0` |
| `path.min_rotation_rad` | double, rad | `0.02` | `>=0` |
| `path.line_width_m` | double, m | `0.05` | `>0` |
| `path.color_rgba` | double[4] | `[0.10,0.80,1.00,1.00]` | 각 값 `[0,1]` |
| `markers.vehicle_scale_m` | double[3], m | `[1.6,0.8,0.4]` | 길이 3, 모두 `>0` |
| `markers.gps_scale_m/lidar_scale_m` | double, m | `0.35/0.35` | `>0` |
| `markers.gps_color_rgba` | double[4] | `[0.10,0.90,0.20,0.90]` | `[0,1]` |
| `markers.lidar_color_rgba` | double[4] | `[1.00,0.50,0.10,0.90]` | `[0,1]` |
| `markers.fault_color_rgba` | double[4] | `[1.00,0.10,0.10,1.00]` | `[0,1]` |
| `panel.refresh_rate_hz` | double, Hz | `5.0` | `>0` |
| `panel.stale_display_sec` | double, s | `1.0` | `>0` |
| `panel.korean_labels.<STATE>` | string | 6개 한국어 상태 | 모두 비어 있지 않아야 패널 구성 성공 |

GPS/LiDAR marker는 마지막 pose를 timeout으로 삭제하지 않습니다. 패널 상태와 함께 해석해야 합니다.

## `bringup.launch` 인자와 정확한 실행

사용 가능한 인자는 다음 명령으로 확인합니다.

```bash
source /opt/ros/noetic/setup.bash
source /home/stier/Mando_1-5/localization/devel/setup.bash
roslaunch mando_localization bringup.launch --ros-args
```

### 기능 인자

| 인자 | 기본값 |
|---|---:|
| `start_imu_driver`, `start_gps_driver` | `false`, `false` |
| `motion_source` | `encoder` |
| `start_interface_adapter`, `start_imu_normalizer`, `start_local_ekf` | `true`, `true`, `true` |
| `enable_gps_fusion`, `enable_lidar_localization` | `true`, `false` |
| `start_global_ekf`, `start_supervisor`, `start_output_gate` | 모두 `true` |
| `start_static_tf_publisher`, `start_visualization`, `start_rviz` | 모두 `true` |

`start_status_manager`는 기존 launch 호출 호환용 별칭이며 새 설정은 `start_supervisor`를 사용합니다. Supervisor를 끄면 내부 Coordinator도 함께 꺼지므로 GPS gate pose가 공개 Global EKF 입력으로 전달되지 않습니다. `motion_source`의 현재 구현 경로는 `encoder`뿐입니다. 다른 문자열은 encoder adapter가 시작되지 않아 상태가 유효해지지 않습니다.

### 교체 가능한 파일 인자

`interfaces_config`, `imu_driver_config`, `gps_driver_config`, `encoder_config`, `tf_config`, `local_ekf_config`, `global_ekf_config`, `gps_reference_config`, `lidar_config`, `status_policy_config`, `relocalization_policy_config`, `visualization_config`, `rviz_config`, `map_file`을 절대 경로로 교체할 수 있습니다.

```bash
roslaunch mando_localization bringup.launch \
  interfaces_config:=/absolute/path/localization_interfaces.yaml \
  tf_config:=/absolute/path/tf_configuration.yaml \
  status_policy_config:=/absolute/path/status_policy.yaml \
  start_rviz:=false
```

실제 node name은 `imu_driver_node_name`, `gps_driver_node_name`, `imu_normalizer_node_name`, `interface_adapter_node_name`, `encoder_adapter_node_name`, `local_ekf_node_name`, `gps_quality_gate_node_name`, `lidar_quality_gate_node_name`, `map_server_node_name`, `amcl_node_name`, `global_ekf_node_name`, `supervisor_node_name`, `output_gate_node_name`, `static_tf_publisher_node_name`, `visualization_node_name`, `rviz_node_name`으로 바꿉니다.

## 현재 환경·운영 주의

- 2026-08-31 재확인에서도 이 호스트의 `rospack find ublox_gps`, `nmea_navsat_driver`는 실패합니다. 설치 전 `start_gps_driver:=true`를 사용하면 안 됩니다.
- IMU는 Xsens serial `DB8GG04M`을 `/dev/imu`로 고정하고, GPS는 `/dev/mando_gps`를 사용합니다. 재연결 후 `readlink -f`로 실제 포트를 검증해야 합니다.
- GPS UART1의 NMEA 출력은 확인됐지만 no-fix이며 ROS 토픽과 UBX 전환은 아직 확인하지 못했습니다.
- `maps/map.yaml`은 저장소에 없고 `map.yaml.example`만 있습니다.
- Xsens raw covariance는 0이지만 현재 `measured` override가 활성화되어 정규화 출력에는 양의 covariance가 생성됩니다. 형식상 통과할 수 있지만 정지·모터 OFF noise floor라 주행 조건에서 `verified`된 값은 아닙니다.
- 센서 정적 TF, GPS datum과 엔코더 거리·조향은 실측 전이며 IMU covariance도 운영 검증 전 상태입니다.
- 기본 RViz display 토픽은 `.rviz`에 고정되어 있어 YAML 값을 런타임 치환하지 못합니다. 공개 토픽을 바꾸는 차량은 `rviz_config` 인자로 별도 `.rviz` 파일을 함께 전달해야 합니다.
