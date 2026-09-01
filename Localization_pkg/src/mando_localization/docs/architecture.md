# 위치추정 아키텍처

![전체 아키텍처](images/localization_architecture.svg)

## 설계 경계

빠르게 갱신되는 상대 움직임과 지도 기준 절대 위치를 분리합니다. Local EKF는 `odom`에서 끊김 없는 움직임을 만들고, Global EKF는 승인된 GPS·AMCL 위치로 `map` 기준 드리프트를 보정합니다. 품질 게이트, 재정합 Coordinator, 상태 Manager, Supervisor와 최종 Output Gate는 서로 다른 책임을 가집니다.

```text
map -- Global EKF --> odom -- Local EKF --> base_link
```

- Local EKF 유일 TF: `odom -> base_link`
- Global EKF 유일 TF: `map -> odom`
- AMCL: `tf_broadcast: false`
- Xsens 드라이버: `pub_transform: false`
- Supervisor(Status Manager·Relocalization Coordinator 포함), Output Gate, 인터페이스 어댑터, 시각화 노드: TF 미발행

## 공개 인터페이스와 고정 내부 연결

`LocalizationInterfaceAdapter`는 계산하지 않는 양방향 relay입니다. 차량별 공개 토픽은 YAML에서 바꿀 수 있지만 외부 패키지와 연결되는 내부 토픽은 고정합니다.

| 방향 | 고정 내부 토픽 | 공개 토픽 기본값 |
|---|---|---|
| Xsens 드라이버 → 공개 IMU | `/mando_localization/internal/driver/imu` | `/molit/sensors/imu/data` |
| u-blox 드라이버 → 공개 GPS | `/mando_localization/internal/driver/gps_fix` | `/molit/sensors/gps/fix` |
| u-blox 드라이버 → 공개 NavPVT | `/mando_localization/internal/driver/gps_navpvt` | `/molit/sensors/gps/navpvt` |
| 정규화 IMU → 두 EKF | `/mando_localization/internal/ekf/imu` | `/molit/localization/imu/normalized` |
| 엔코더 Twist → 두 EKF | `/mando_localization/internal/ekf/twist` | `/molit/vehicle/twist` |
| Coordinator 승인 GPS → Global EKF | `/mando_localization/internal/ekf/gps_pose` | `/molit/localization/gps/map_pose` |
| LiDAR 게이트 → Global EKF | `/mando_localization/internal/ekf/lidar_pose` | `/molit/localization/lidar/map_pose` |
| Local EKF → 공개 출력 | `/mando_localization/internal/ekf/local_odometry` | `/molit/localization/local/odometry` |
| Global EKF → 공개 출력 | `/mando_localization/internal/ekf/global_odometry` | `/molit/localization/global/odometry` |
| map_server → 공개 지도 | `/mando_localization/internal/amcl/map` | `/map` |
| RViz 수동 초기값 → AMCL | `/mando_localization/internal/amcl/initialpose` | `/initialpose` |

드라이버를 launch에서 시작하지 않으면 외부 드라이버가 공개 IMU/GPS 토픽을 직접 발행할 수 있습니다. 인터페이스 어댑터가 자기 구독하지 않도록 드라이버를 시작할 때만 고정 driver 토픽을 사용합니다. NavPVT는 `topic_tools/ShapeShifter`로 실제 드라이버 메시지 타입을 바꾸지 않고 relay합니다. EKF·AMCL 내부 토픽과 output remap은 설정 중복을 막기 위한 패키지 계약이므로 차량별 YAML로 변경하지 않습니다.

GPS gate는 Local innovation 전 quality candidate와 정상 gate pose를 각각
`/mando_localization/internal/gps/candidate_pose`,
`/mando_localization/internal/gps/gate_pose`로 Coordinator에 보냅니다.
Coordinator만 `/molit/localization/gps/map_pose`를 발행하며, 인터페이스
어댑터가 이 공개 승인 결과만 Global EKF 내부 토픽으로 relay합니다.

## 처리 단계

### 1. IMU·엔코더와 Local EKF

1. Xsens raw IMU는 공개 토픽으로 relay됩니다.
2. `ImuNormalizer`가 timestamp, frame, quaternion, covariance와 finite 값을 검사해 정규화 IMU를 발행합니다. covariance override가 비활성이면 raw 세 covariance 배열의 양수 대각을 요구합니다. 현재는 `measured` override가 활성화되어 정지 bag에서 측정해 설정한 표준편차의 제곱으로 출력 covariance를 교체합니다. Override는 비어 있지 않은 측정 시각·출처와 세 축의 양수 표준편차가 모두 있어야 켤 수 있습니다.
3. `EncoderToTwistAdapter`는 Arduino의 `/erp42_serial/feedback` (`erp42_msgs/SerialFeedBack`)을 직접 구독하고 직전 `alive` 값과 달라졌는지, `speed`·`steer`의 finite 여부, `|speed| <= 30 m/s`, `|encoder| <= 100000`, `steer` ADC 0–65535 범위를 검사합니다. 이 메시지에는 Header가 없으므로 PC 수신 시각과 `base_link`를 출력 Twist header에 명시합니다.
4. `speed * speed_scale * direction_sign`만 `Twist.linear.x`로 사용합니다.
5. `steer`, `brake`, `encoder`는 현재 상태·일관성 진단용입니다. 조향 ADC로 `Twist.angular.z`를 계산하지 않습니다.
6. Local EKF는 `linear.x`, IMU yaw와 yaw rate를 융합해 Local Odometry와 `odom -> base_link`를 만듭니다.

`Twist`는 차량의 위치가 아니라 순간 속도 표현입니다. `linear.x/y/z`는 선속도, `angular.x/y/z`는 각속도이며 이 패키지는 covariance가 포함된 `TwistWithCovarianceStamped`를 사용합니다. 엔코더가 채우는 관측은 `linear.x`뿐이고, 회전 속도 `angular.z`는 Twist 어댑터가 임의 계산하지 않으며 IMU yaw rate가 EKF의 별도 입력으로 들어갑니다. 관측하지 않는 Twist 축은 큰 분산으로 표시합니다.

Global EKF도 동일한 정규화 IMU와 Twist를 직접 받습니다. Local Odometry를 다시 융합하면 상관된 측정을 독립 정보처럼 중복 사용할 수 있으므로 Global EKF 입력으로 넣지 않습니다.

### 2. GPS 직접 투영과 품질 게이트

GPS 경로에는 `robot_localization/navsat_transform_node`가 없습니다. `OdometryGpsFusion`이 다음 순서로 직접 처리합니다.

1. `NavSatFix.status`, `gps_link` frame, 0/역행/과거/미래 timestamp, 위·경도·고도와 covariance의 유한성·대칭성·양의 준정부호성을 검사합니다.
2. `first_fix` 또는 측량된 `manual_datum`을 기준으로 WGS84 타원체의 자오선·묘유선 곡률을 사용해 datum 주변 east/north 거리를 계산합니다.
3. `yaw_offset_rad`로 ENU를 회전하고 `map_x/y/z_m` 오프셋을 더해 `map` pose를 만듭니다.
4. Local Odometry가 receipt/stamp timeout 안이고 `odom`/`base_link` 계약과 finite covariance를 만족하는지 확인합니다.
5. 직전 승인 GPS와 그때의 Local Odometry를 anchor로 저장합니다. 현재 Local Odometry 증분으로 map 위치를 예측한 뒤 유클리드 거리와 결합 covariance 기반 Mahalanobis innovation을 검사합니다.
6. fix/frame/time/covariance와 투영을 통과한 quality candidate는 Global EKF와 분리된 내부 토픽으로 `RelocalizationCoordinator`에 전달합니다.
7. 정상·짧은 단절에서는 기존 Local innovation, 후보 step과 연속 횟수를 통과한 gate pose만 Coordinator가 공개하고, 인터페이스 어댑터가 Global EKF 내부 토픽으로 relay합니다.

장기 단절은 이전 anchor가 있고 마지막 gate 승인 receipt 후 2초를 **초과한**
뒤 새 candidate가 들어올 때 별도 처리합니다. Fresh LiDAR가 있으면 각 pair의
timestamp 차이 0.20초와 XY 거리 5 m 이내를 확인하고, 군집 첫 GPS 후보에서
2 m 안인 증가 timestamp GPS candidate의 LiDAR 비교가 3회 연속이어야 합니다. Mismatch나 중복·역행
timestamp는 카운트를 초기화합니다. 이후 `GpsGateReanchor` transaction 명령을
보내고 GPS gate가 prediction anchor를 실제 적용했다는 동일 transaction ACK를
받아야만 GPS pose를 다시 공개합니다. Gate는 최신 candidate와 명령이 2 m 이내,
stamp 차이 1초 이내이고 Local Odometry가 fresh한지도 확인합니다. Coordinator는
ACK의 transaction ID, stamp와 XY가 요청과 일치해야 완료합니다. LiDAR pose가
Global EKF의 구성 입력이므로 코드상 강제 reset은 하지 않지만, 해당 측정의 EKF
내부 수용 ACK를 확인한 것은 아닙니다. 증가 여부는 GPS candidate timestamp에만
적용되며 같은 LiDAR pose도 freshness·skew 안에서는 여러 비교에 재사용될 수
있습니다. fresh·time-aligned LiDAR pair를 사용할 수 없을 때는 정지 중 증가
timestamp GPS 후보 5회, fresh Global Odometry와 `reference.measured=true`를
요구하고 Global EKF `set_pose` 뒤 서로 다른 증가 timestamp 결과가 3회 연속
일치하는지와 GPS gate ACK까지 확인합니다. Coordinator는 datum mode·source를
다시 검증하지 않고 post-reset 결과도 XY만 확인합니다. GPS-only 자동 reset은
기본 비활성입니다.

최초 위치도 기본 3회 연속 정상 fix가 필요합니다. 한 번 승인된 GPS가 invalid
fix·시간 gap·innovation 거부 뒤 새 후보를 평가할 때 GPS gate는
`/mando_localization/internal/gps/gate_relocalizing=true`를 냅니다. Coordinator가
이 단기 gate 상태와 장기 복구 상태를 합쳐 최종
`/mando_localization/internal/gps/relocalizing`을 발행합니다. 메시지가 완전히
멈춘 경우에는 Manager의 GPS timeout과 bounded dead reckoning 정책이 먼저
동작합니다. GPS status와 covariance 처리는
[GPS 품질·재연결 문서](gps_quality_and_recovery.md)에 자세히 설명합니다.

`first_fix`는 매 실행의 첫 유효 위치가 `(0,0,0)`이 되는 개발 모드입니다. 실제 OccupancyGrid와 함께 쓸 때는 `manual_datum`, 측량 WGS84 좌표, 지도상의 datum 좌표와 yaw가 모두 필요합니다. 이 투영은 소규모 주행 영역용 local tangent 근사이며 UTM 변환이 아닙니다.

> 현재 GPS 게이트는 `NavSatFix.header.frame_id`가 `gps_link`인지 확인하지만 TF를 이용한 안테나 lever-arm 보정은 수행하지 않습니다. 정밀 차량 기준점 위치가 필요하면 datum 정의와 별도로 lever-arm 보정 구현을 검증해야 합니다.

### 3. LaserScan·AMCL·LiDAR 게이트

LiDAR 경로는 기본 비활성입니다.

1. `map_server`가 지정된 실제 `map.yaml`을 `/mando_localization/internal/amcl/map`으로 발행합니다. AMCL이 이 고정 토픽을 직접 사용하고, 인터페이스 어댑터가 RViz용 공개 `/map`으로 latched relay합니다.
2. LiDAR 게이트가 원시 `/molit/sensors/lidar/scan`의 frame, timestamp, 각도·거리 메타데이터, point 수, finite 비율과 NaN을 검사합니다.
3. 통과한 scan만 `/mando_localization/internal/amcl/scan`으로 relay합니다.
4. AMCL은 scan과 map을 정합하고 `/mando_localization/internal/amcl/pose`를 냅니다. TF는 발행하지 않습니다.
5. LiDAR 게이트는 AMCL pose의 map frame, timestamp, quaternion, 위치/yaw covariance의 유한성·대칭성·양의 준정부호성을 검사합니다.
6. 이미 승인된 pose 부근의 작은 이동은 즉시 통과합니다. 큰 위치/yaw 점프나 단절 후 복귀는 설정 반경 안의 pose가 기본 3회 연속 모여야 통과합니다.
7. 복구 중에는 `/mando_localization/internal/lidar/relocalizing=true`를 발행합니다.

GPS-assisted AMCL 초기 위치는 `reference.require_measured_datum_for_lidar=true`일 때 `reference.measured=true` 플래그를 만족해야 최초 한 번 고정 내부 `/mando_localization/internal/amcl/initialpose`로 발행합니다. 이 검사도 datum mode·source까지 보장하지 않습니다. 단독 GPS에는 heading이 없으므로 기본 yaw 0은 약 `pi² rad²`의 큰 분산을 가진 중립 초기값일 뿐입니다. 자동 초기화가 비활성/실패한 경우 RViz `2D Pose Estimate`가 공개 `/initialpose`를 발행하고 인터페이스 어댑터가 같은 내부 토픽으로 relay하는 수동 대안입니다.

### 4. Global EKF와 TF

Global EKF의 2D 관측은 다음과 같습니다.

| 입력 | 융합 상태 |
|---|---|
| 엔코더 Twist | `vx` |
| IMU | `yaw`, `vyaw` |
| GPS pose | `x`, `y` |
| LiDAR/AMCL pose | `x`, `y`, `yaw` |

Global EKF raw 출력은 먼저 고정 내부 토픽으로 나오고 인터페이스 어댑터가 `/molit/localization/global/odometry`로 relay합니다. 이 raw 출력은 상태 Manager와 Output Gate의 입력이며, `/molit/localization/odometry`가 안전 검사를 마친 공개 최종 출력입니다.

### 5. 상태 판단과 안전 출력

`LocalizationSupervisor` 프로세스는 세 구성요소를 함께 실행합니다.

- `LocalizationStatusManager`: 다음 입력을 관찰해 내부 evaluated state/valid/status만 계산하며 pose를 계산·중계하지 않습니다.
- `RelocalizationCoordinator`: GPS gate의 승인 전 candidate, LiDAR pose, Twist와 Global Odometry를 이용해 장기 단절 복구를 조정합니다.
- `LocalizationSupervisor`: 두 결과를 중재하며 공개 `/molit/localization/state`, `/valid`, `/status`를 단독 발행합니다. evaluated state/valid 또는 recovery-active heartbeat가 누락·0.50초 stale이면 `FAULT`, recovery가 active이면 `RELOCALIZING`으로 fail-closed합니다. evaluated diagnostic status의 freshness는 공개 DiagnosticArray 구성에 사용합니다.

Status Manager와 Coordinator는 별도 ROS node가 아니라
`/localization_supervisor` 프로세스 내부 C++ 객체입니다. 기본 bringup의
`rosnode list`에는 Supervisor 하나만 나타납니다.

Status Manager가 관찰하는 항목은 다음과 같습니다.

- 정규화 IMU, Arduino SerialFeedBack, Twist, Local/Global Odometry의 receipt/stamp age, frame, finite 값과 covariance. 이 중 IMU·SerialFeedBack·Twist·Local Odometry가 local motion이고 Global Odometry는 별도 출력 건강도로 판정합니다.
- GPS/LiDAR 승인 pose의 age와 covariance
- GPS/LiDAR 각각의 relocalizing 신호
- GPS↔LiDAR 거리와 각 절대 pose↔Global Odometry 거리
- USB 장치 경로와 예상 드라이버 node 존재 여부(진단 표시용)
- 마지막 절대 위치 이후 시간과 Global Odometry 이동거리

Output Gate는 `valid=true`가 최근에 수신됐는지 다시 확인하고 Global Odometry의 timestamp, `map/base_link` frame, 위치·quaternion·twist, pose/twist covariance의 유한성·대칭성·양의 준정부호성과 위치 분산을 검사합니다. 하나라도 실패하면 메시지를 내보내지 않으며 마지막 pose를 새 timestamp로 재발행하지 않습니다.

### 6. RViz와 MarkerArray

시각화 노드는 최종 Odometry가 유효할 때만 Path에 점을 추가합니다. 기본 MarkerArray는 차량 cube, GPS sphere, LiDAR sphere와 path line strip입니다. 유효성 하강 시 Path는 지워지고 차량은 다음 발행부터 fault 색으로 표시됩니다.

RViz 기본 한 창은 다음을 동시에 표시합니다.

- `/map`, TF tree와 기준 격자
- 원시 LaserScan
- 최종 Odometry와 위치·방향 covariance
- Path와 MarkerArray
- GPS·LiDAR 승인 pose와 covariance
- `/initialpose`를 발행하는 `2D Pose Estimate`
- 상태·유효성·DiagnosticArray를 읽기만 하는 한국어 패널

GPS/LiDAR marker는 마지막 수신 pose를 유지하므로 센서가 끊겨도 화면에 남을 수 있습니다. 현재 건강 여부는 반드시 오른쪽 상태 패널과 함께 해석해야 합니다.

상태 패널은 `/mando_localization/interfaces`와 `/mando_localization/visualization` 전역 파라미터를 읽습니다. 따라서 단독 RViz가 필요할 때도 `roslaunch mando_localization visualization.launch`로 시작해야 하며, `rviz -d ...`만 실행하면 `인터페이스 YAML 미로딩`이 표시될 수 있습니다.

## 운영 전 준비 상태

현재 저장소에는 실지도 `maps/map.yaml`이 없고, 센서 정적 TF 세 개는 모두 비활성·미측정입니다. GPS datum도 `first_fix`, 엔코더 거리·조향 보정은 미측정 상태입니다. IMU covariance는 정지·모터 OFF noise floor만 `measured`이고 주행 조건에서는 미검증입니다. 이 기본값은 아키텍처 시험용이지 실차 정밀 위치의 실측 증거가 아닙니다.

Xsens는 2026-08-30 실측에서 `/dev/ttyUSB0`, serial `DB8GG04M`, device ID `03889250` (`MTi-3-8A7G6`), 115200 baud로 auto-scan됐고 IMU 약 100.95 Hz, `imu_link`, timestamp 존재까지 확인했습니다. 원본 세 covariance 배열은 모두 0이지만 2026-08-31 정지·모터 OFF bag으로 측정한 표준편차 override가 활성화돼 정규화 출력에는 양의 covariance가 들어갑니다. 다만 정지 noise floor이므로 주행·모터 진동·자기장 교란과 절대 yaw 정확도를 보증하지 않습니다.

GPS는 `/dev/ttyUSB1`, serial `DBT042C1`, 460800 baud에서 NMEA `GNRMC/GNGGA` 출력까지 확인했지만 `V/0` no-fix 상태였습니다. 현재 YAML은 드라이버 시작 시 휘발성 RAM/포트 설정을 UBX로 전환하고 Flash 저장은 하지 않습니다. 2026-08-31 재확인에서도 `ublox_gps`, `/dev/mando_gps`가 없어 ROS `NavSatFix`·`NavPVT`와 실제 UBX 전환 결과는 아직 검증되지 않았습니다.
