# 실차 Hardware Calibration Checklist

이 문서는 `real_mode:=true calibration_required:=false`를 허용하기 전에 작성하는
측정 기록이다. 현재 `vehicle.yaml`, `scan_leveling.yaml`, `rtk_gate.yaml`,
`timeouts.yaml` 값은 시작용 sample이며 실제 1/5 차량의 승인값이 아니다.

모든 항목에 측정값, 측정 방법, 원본 log/rosbag 경로, 담당자, 날짜를 남긴다. 하나라도
미완료이면 `calibration_required:=true`를 유지한다. 센서 모델이 바뀌거나 장착 위치,
타이어, steering linkage, firmware가 바뀌면 영향을 받는 항목을 다시 측정한다.

## 1. 시험 구성 기록

| 항목 | 기록 |
|---|---|
| 차량 ID / chassis revision | |
| NUC 또는 노트북 / Ubuntu / ROS version | |
| LiDAR model / serial / firmware | |
| IMU model / serial / firmware | |
| GNSS receiver·antenna / firmware | |
| RTK correction source / NTRIP mountpoint | |
| Camera model / serial / mode | |
| Encoder·motor controller·steering sensor model | |
| Tire diameter·pressure / 차량 적재 상태 | |
| Adapter package / Git commit / config revision | |
| 시험 rosbag와 결과 문서 절대 경로 | |

## 2. 좌표계와 normalized 입력

ROS REP-103 기준 `base_link`는 x 전방, y 좌측, z 위쪽으로 둔다. frame 이름에는
leading `/`를 쓰지 않는다. Vendor adapter가 다음 경계에서 단위와 부호를 변환하고,
core node에는 이미 정규화된 값만 전달해야 한다.

- [ ] `/slam/input/vehicle_speed`의 `linear.x`는 m/s, 전진 양수·후진 음수다.
- [ ] `/slam/input/steering_angle`은 실제 road-wheel angle rad, 좌회전 양수다.
- [ ] IMU angular velocity는 rad/s이고 orientation convention과 축 방향을 기록했다.
- [ ] GNSS는 WGS84 `NavSatFix`와 m² 단위 covariance를 제공한다.
- [ ] RTK adapter의 `DiagnosticStatus.message`는 정확히 `FIX`, `FLOAT`, `NO_FIX`다.
- [ ] 모든 stamped topic은 같은 clock domain의 양수·유한 source timestamp를 보존한다.
- [ ] 실 supplier와 rosbag replay supplier가 같은 normalized topic을 동시에 publish하지
  않음을 `rostopic info`로 확인했다.

## 3. Chassis, steering, encoder, and speed

`vehicle_odometry_node`는 rear-axle 기준 planar Ackermann model을 사용한다.
`wheel_encoder_adapter_node`는 표준 `JointState`의 wheel rad/rad/s에 `encoder.yaml`의
반지름과 방향 부호를 적용한다. Vendor tick 또는 motor RPM은 그 전에 별도 hardware
adapter에서 wheel rad/rad/s로 정규화해야 한다. Steering servo scale/offset도 vendor
adapter 또는 별도 hardware profile의 책임이다.

| 측정 항목 | 방법 | 결과 / 불확도 | 적용 위치 | 승인 |
|---|---|---|---|---|
| `wheelbase_m` | 좌·우 axle center를 각각 측정하고 평균 | | `vehicle.yaml` | [ ] |
| Steering zero offset | 직진 정렬 후 반복 측정 | | adapter | [ ] |
| Steering sign | 좌/우 명령과 road-wheel 방향 비교 | | adapter | [ ] |
| Servo-to-road-wheel ratio/curve | 좌·중앙·우 여러 지점에서 실측 | | adapter | [ ] |
| Steering backlash/hysteresis | 양 방향 접근값 비교 | | covariance/limit | [ ] |
| `max_abs_steering_rad` | 기구적 stop보다 안전 margin 안쪽 | | `vehicle.yaml` | [ ] |
| Encoder ticks per wheel revolution | wheel 공회전 및 지상 반복 | | adapter | [ ] |
| Motor RPM to wheel speed scale | 직선 실거리·시간 기준과 비교 | | adapter | [ ] |
| `wheel_radius_m` | 실측 반지름 후 직선 실거리와 비교 | | `encoder.yaml` | [ ] |
| Left/right wheel sign | 전진·후진에서 양쪽 부호 비교 | | `encoder.yaml` | [ ] |
| Vehicle speed scale | 전진/후진, 저속/중속 별 비교 | | adapter/`encoder.yaml` | [ ] |
| Speed sign and zero deadband | 전진·정지·후진 반복 | | adapter/`encoder.yaml` | [ ] |
| Speed/steering source latency | source time 대 receipt time 분포 | | adapter/timeout | [ ] |
| Pose/twist covariance | 반복 시험 오차 분산으로 산정 | | `vehicle.yaml` | [ ] |
| Encoder speed covariance | 기준 속도 대비 반복 오차 분산 | | `encoder.yaml` | [ ] |

필수 동적 확인:

- [ ] 평지 직진 왕복에서 wheel odometry의 lateral/yaw bias를 기록했다.
- [ ] 일정 steering 원주행을 좌·우 각각 수행해 회전반경
  `R = wheelbase / tan(steering)`과 비교했다.
- [ ] 후진에서 위치와 yaw 부호가 물리 운동과 일치한다.
- [ ] Speed와 steering sample의 실제 pairing 지연이 현재
  `control_pair_timeout_sec: 0.10` 이내인지 측정했다.
- [ ] 설정한 `max_abs_speed_mps`와 `max_abs_steering_rad`가 정상 입력을 자르지 않으면서
  잘못된 단위 입력은 차단한다.

## 4. Sensor extrinsics

물리 TF owner는 URDF 또는 하나의 static TF supplier로 단일화한다. Driver가 같은
edge를 publish하는지 함께 확인한다. 변환은 자/각도계 또는 calibration 결과와
불확도를 함께 기록한다.

| Transform / 항목 | x y z (m) | roll pitch yaw (rad) | 측정 방법·불확도 | 승인 |
|---|---|---|---|---|
| `base_link -> lidar_link` | | | | [ ] |
| `base_link -> imu_link` | | | | [ ] |
| `base_link -> gps_link` antenna phase-center | | | | [ ] |
| `base_link -> camera_link` optical convention 포함 | | | | [ ] |

- [ ] `base_link` 원점과 rear axle 기준의 관계를 그림과 수치로 남겼다.
- [ ] LiDAR 값은 `scan_leveling.yaml`의 `mount_xyz_rpy` 순서
  `[x, y, z, roll, pitch, yaw]`에 반영했다.
- [ ] IMU orientation이 vehicle body의 roll/pitch/yaw와 같은 부호인지 정지·기울임
  시험으로 확인했다.
- [ ] IMU quaternion norm, orientation covariance, gyro bias와 warm-up 시간을 기록했다.
- [ ] GNSS lever arm을 기록하고, antenna/receiver가 publish하는 frame과
  `gps_link`가 일치한다.
- [ ] Camera intrinsics와 distortion은 별도 calibration file로 보존했으며 image의
  frame/time이 TF 및 다른 센서와 일치한다.

## 5. Timestamp, cadence, and timeout

각 센서에 대해 최소 5분 이상 기록하고 rate, dropout, transport latency의
최소/중앙/p95/최대값을 계산한다. `rostopic hz`만으로 source timestamp 품질을 대신하지
않는다. Steering angle과 RTK status는 현재 unstamped message이므로 adapter callback과
ROS receipt time의 한계를 별도로 기록한다.

| Topic | 예상 시작 timeout | 관측 Hz min/median/p95 | source 지연 p95/max | dropout max | 승인 |
|---|---:|---|---|---|---|
| `/slam/input/scan_raw` | 0.25 s | | | | [ ] |
| `/slam/input/imu` | 0.15 s | | | | [ ] |
| `/slam/input/gnss/fix` | 1.50 s | | | | [ ] |
| `/slam/input/gnss/rtk_status` | 1.50 s | | receipt 기준 | | [ ] |
| `/slam/input/vehicle_speed` | 0.25 s | | | | [ ] |
| `/slam/input/steering_angle` | 0.25 s | | receipt 기준 | | [ ] |
| `/slam/input/camera/front/image_raw` | 0.25 s | | | | [ ] |

- [ ] Header stamp가 한 replay epoch 안에서 단조 증가하고 0 또는 미래
  시간이 없다. Bag loop/rewind로 시간이 뒤로 가면 각 stateful node가 state를
  reset하고 recovery 상태로 전환하는지 별도로 확인한다.
- [ ] Sensor 간 offset과 jitter가 scan/IMU `sync_slop_sec: 0.05` 안에 들어온다.
- [ ] 측정 결과를 근거로 `timeouts.yaml`을 조정하고 정상·stale·recovery 진단을 모두
  확인했다.
- [ ] CPU 부하 상태에서도 callback rate와 latency가 허용 범위에 있다.

## 6. RTK-GNSS and recovery

| 시험 | 기록할 값 | 결과 | 승인 |
|---|---|---|---|
| NTRIP 연결 전 / correction 중단 | vendor state와 `NO_FIX` mapping | | [ ] |
| Ambiguity 미고정 | vendor state와 `FLOAT` mapping | | [ ] |
| cm급 고정 | vendor state와 `FIX` mapping | | [ ] |
| FIX covariance | x/y m² 분포, 현재 gate 0.04 m² 이하 여부 | | [ ] |
| 정지 위치 반복 | CEP/표준편차/튀는 sample | | [ ] |
| 주행 중 blackout | local odometry 지속성과 drift | | [ ] |
| FIX 복구 | innovation, pose jump, 수용 시간 | | [ ] |

- [ ] Vendor code를 문자열로 추측하지 않고 receiver 문서와 실제 log로 mapping했다.
- [ ] `FLOAT`와 `NO_FIX`가 `/slam/gnss/fix_accepted`로 전달되지 않는다.
- [ ] 유효한 `FIX`도 stale state, 알 수 없는/음수/큰 covariance, 비현실적 innovation이면
  reject되는 것을 확인했다.
- [ ] RTK loss가 local EKF와 LiDAR localization을 즉시 중단시키지는 않지만 진단에는
  명확히 나타난다.

## 7. 경사로, LiDAR leveling, and 2D limits

현재 map과 local EKF는 `two_d_mode: true`이며 도로 높이를 추정하지 않는다. LiDAR
cloud만 IMU roll/pitch와 측정된 mount transform으로 level한다. IMU yaw는 leveling
회전에서 의도적으로 제외된다. 시작값은 최대 roll/pitch 0.5236 rad (30°), 출력 z
범위 -0.50~0.50 m다.

- [ ] 평지 정방향/역방향에서 같은 벽·연석의 leveled z 분포를 기록했다.
- [ ] 오르막 진입, 오르막 정상, 내리막 진입, 내리막 종료를 각각 양 방향으로 시험했다.
- [ ] 경사 변화에서 map wall이 이중으로 생기거나 curb/장애물이 z filter로 사라지지
  않는지 확인했다.
- [ ] 실제 최대 roll/pitch와 IMU 진동 peak가 `max_abs_tilt_rad`보다 안전 margin 안에
  있다. 초과 시 임의로 제한을 키우지 않고 장착/좌표/필터를 먼저 점검했다.
- [ ] `min_z_m`, `max_z_m`를 ground, curb, cone/장애물의 실측 분포로 결정했다.
- [ ] 오르막과 내리막 각각 반복 map의 벽·연석 정렬 오차와 loop closure 오차를
  수치로 남겼다.

## 8. 최종 acceptance와 sign-off

| Acceptance metric | 요구 기준 | 측정 결과 / bag·report | 승인 |
|---|---|---|---|
| Normalized topic type/frame/unit | README 계약과 100% 일치 | | [ ] |
| TF owner | 각 edge publisher 1개 | | [ ] |
| Sensor health | 정상 OK, disconnect STALE/MISSING, 복구 OK | | [ ] |
| Wheel straight/circle/reverse error | 팀이 정한 수치 기준 | | [ ] |
| RTK FIX acceptance / false acceptance | 팀이 정한 수치 기준 | | [ ] |
| Flat/uphill/downhill map repeatability | 팀이 정한 수치 기준 | | [ ] |
| Localization drift / relocalization | 팀이 정한 수치 기준 | | [ ] |
| CPU, memory, pose rate, latency | 대회 노트북 여유 포함 기준 | | [ ] |
| Emergency stop / sensor loss behavior | 안전 담당 승인 | | [ ] |

최종 승인:

- [ ] 위 표의 수치 기준이 시험 전에 정해졌고 결과가 모두 충족됐다.
- [ ] 적용 YAML/adapter commit과 원본 rosbag이 read-only 위치에 보존됐다.
- [ ] Mapping과 localization을 각각 동일 설정으로 재실행해 결과를 재현했다.
- [ ] 담당자 1: 이름 / 날짜 / 서명
- [ ] 독립 검토자: 이름 / 날짜 / 서명

두 서명 뒤에만 해당 hardware profile에서
`real_mode:=true calibration_required:=false`를 사용한다. 합성 smoke 통과만으로 이
승인을 대체할 수 없다.
