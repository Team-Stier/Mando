# GPS 품질, covariance와 재연결 판단

이 문서는 `sensor_msgs/NavSatFix`가 들어온 뒤 GPS gate, 장기 단절 복구,
Global EKF까지 이어지는 현재 구현 계약을 설명합니다. 설정 수치는 수신기의 실제
정확도 측정값이 아니라 메시지를 사용할지 결정하는 한계값입니다.

## GPS 데이터의 책임 경계

| 단계 | 책임 | 만들지 않는 것 |
|---|---|---|
| GPS 수신기·드라이버 | fix 종류, 위·경도·고도, timestamp, `position_covariance` 발행 | map 위치 승인 |
| `OdometryGpsFusion` | fix/frame/time/covariance 검사, WGS84→map 투영, 단기 Local innovation gate | 장기 단절의 최종 승인 |
| `RelocalizationCoordinator` | 정상 gate pose relay, 장기 단절에서 LiDAR/GPS-only 복구 조정, 공개 GPS pose 단일 발행 | 센서 covariance 자체 추정 |
| Global EKF | 승인 GPS의 x/y와 다른 관측을 융합 | GPS 품질 gate 대체 |

공개 `/molit/localization/gps/map_pose`의 단일 발행자는
`RelocalizationCoordinator`입니다. GPS gate가 만드는 quality candidate와 정상
gate pose는 각각 내부 토픽으로만 Coordinator에 전달되며 Global EKF를 직접
우회하지 않습니다.

## `NavSatStatus.status` -1, 0, 1, 2

| 값 | ROS 상수 | 의미 | 현재 gate 처리 |
|---:|---|---|---|
| `-1` | `STATUS_NO_FIX` | 유효 위치 fix 없음 | 즉시 거부 |
| `0` | `STATUS_FIX` | 일반 GNSS fix | 추가 품질검사 진행 |
| `1` | `STATUS_SBAS_FIX` | SBAS 보정 fix | 추가 품질검사 진행 |
| `2` | `STATUS_GBAS_FIX` | GBAS 보정 fix | 추가 품질검사 진행 |

현재 `quality.minimum_fix_status: 0`이므로 `0/1/2`는 status 단계만 통과하고
모두 동일한 frame, timestamp, 좌표, covariance와 innovation 검사를 받습니다.
숫자가 크다는 사실만으로 실제 오차가 몇 m인지 정할 수 없으며, u-blox
`NavPVT.fixType`, carrier solution/RTK 상태와도 같은 필드가 아닙니다.

## GPS covariance는 어디에서 오는가

원칙적으로 `NavSatFix.position_covariance`와
`position_covariance_type`은 GPS 드라이버가 수신기 정보를 변환해 제공합니다.
GPS gate나 EKF가 그 값을 GPS 원시 측정에서 새로 계산하는 구조가 아닙니다.

- `COVARIANCE_TYPE_APPROXIMATED`부터 `KNOWN`까지는 메시지의 3×3 ENU
  covariance를 유한성·대칭성·양의 준정부호성 및 분산 상한으로 검사합니다.
- 수평 east/north 분산은 각각 `25 m²` 이하, up 분산은 `100 m²` 이하여야 합니다.
- `COVARIANCE_TYPE_UNKNOWN`이면 메시지 값을 신뢰하지 않고 설정 fallback인
  east/north `25 m²`, up `100 m²`, 교차항 0을 사용합니다.
- fallback은 “실제 오차가 5 m”라는 측정 결과가 아니라 unknown covariance를
  무한 신뢰하지 않기 위한 보수적 설정값입니다.
- ENU covariance는 `yaw_offset_rad`와 함께 map XY 좌표계로 회전된 뒤 pose
  covariance가 됩니다.

따라서 status가 `2`여도 covariance·timestamp·frame 또는 위치 일관성이 나쁘면
거부될 수 있고, status가 `0`이어도 모든 추가 검사를 통과하면 후보가 될 수
있습니다.

## 정상·짧은 단절: Local Odometry 비교

GPS gate는 직전 승인 GPS와 당시 Local Odometry를 anchor로 저장합니다. 현재
Local Odometry 증분으로 예상 map 위치를 만든 뒤 다음을 검사합니다.

```text
Euclidean innovation <= 10.0 m
Mahalanobis distance <= 5.0
```

여기서 `10.0 m`는 현재 GPS와 Local의 거리가 항상 10 m라는 뜻이 아니라
`quality.max_position_innovation_m`의 거부 경계입니다. 실제 innovation은 GPS
후보마다 달라집니다. 짧은 구간에서는 Local Odometry가 연속 움직임을 잘
표현하므로 점프·오측정을 거르는 기준으로 사용하지만, 단절이 길어져 drift가
누적된 뒤에는 이 비교만으로 올바른 GPS를 거부할 수 있습니다.

## 장기 단절: Local을 최종 기준으로 쓰지 않는 이유

이전 승인 anchor가 있고 마지막 gate pose 수신 후 정확히 `2.0초`를 **초과한**
뒤 새 quality candidate가 들어오면 Coordinator가 장기 복구를 시작합니다. GPS
callback이 완전히 멈춘 동안에는 Coordinator가 후보를 만들 수 없으므로 Status
Manager의 GPS timeout과 bounded dead reckoning이 먼저 동작합니다.

장기 후보 자체도 `map` frame, age 1.0초 이하, 미래 0.10초 이하, 유효한 XY
covariance를 만족해야 합니다.

### fresh·time-aligned LiDAR map pose를 사용할 수 있을 때

LiDAR localization이 활성화되고 다음 조건을 모두 만족한 증가 timestamp의 GPS
candidate가 3회 연속 필요합니다.

1. LiDAR receipt/stamp age가 0.50초 이하
2. GPS와 LiDAR timestamp 차이가 0.20초 이하
3. 현재 GPS↔LiDAR XY 거리가 5.0 m 이하
4. 각 GPS 후보가 군집 첫 후보에서 2.0 m 이내
5. GPS candidate timestamp가 직전 후보보다 증가

한 번이라도 LiDAR 거리 조건이 깨지거나 중복·역행 timestamp가 들어오면 기존
군집을 버립니다. 현재 카운터가 독립성을 확인하는 대상은 GPS candidate
timestamp입니다. 같은 LiDAR pose도 receipt/stamp freshness와 0.20초 skew를 계속
만족하면 여러 GPS candidate의 비교 기준으로 재사용될 수 있으며, LiDAR
timestamp 증가 자체는 요구하지 않습니다.

LiDAR pose는 Global EKF의 구성된 입력이므로 이 경로의 코드는 Global EKF
`set_pose`를 호출하지 않습니다. 다만 Coordinator는 해당 LiDAR 측정이 EKF 내부
innovation gate에 실제 수용됐다는 ACK를 받지 않습니다. 일반 융합 결과는 이후
LiDAR↔Global 거리 일관성으로 간접 감시할 뿐입니다.

후보 3회가 통과해도 바로 공개하지 않습니다. Coordinator가 0이 아닌 증가
`transaction_id`와 pose를 `GpsGateReanchor` 명령으로 보냅니다. GPS gate는
최신 quality candidate와 명령의 거리 2.0 m 이하, stamp 차이 1.0초 이하,
fresh Local Odometry, map frame·수치·quaternion·covariance를 확인한 뒤 prediction
anchor를 실제 적용합니다. 적용 후 같은 transaction을 ACK하며, Coordinator는
transaction ID, stamp와 XY가 요청과 일치할 때만 GPS pose를 공개합니다.
ACK가 1.0초 안에 없거나 다르면 `RELOCALIZING`, `valid=false`를 유지합니다.

### fresh·time-aligned LiDAR pair를 사용할 수 없을 때

GPS-only 전역 reset 경로는 구현되어 있지만 기본
`automatic_reset_enabled: false`입니다. 활성화하려면 다음 조건이 모두
필요합니다.

1. `reference.measured: true` 플래그
2. fresh Twist와 `|vx| <= 0.30 m/s`
3. first candidate 기준 2.0 m 안의 증가 timestamp GPS 후보 5회
4. fresh Global Odometry
5. Global EKF 전용 `set_pose` 호출 후 목표 2.0 m 안의 서로 다른 증가
   timestamp 결과 3회
6. 동일한 GPS gate reanchor command/ACK

여기서 “사용할 수 없음”은 LiDAR 비활성·invalid·stale뿐 아니라 GPS와 LiDAR의
stamp skew가 0.20초를 넘는 경우도 포함합니다. 반대로 fresh·time-aligned LiDAR와
GPS가 5 m 넘게 다르면 GPS-only로 우회하지 않고 `LIDAR_MISMATCH`로 차단합니다.

Coordinator가 현재 검사하는 datum 조건은 `reference.measured` bool뿐이며
`reference.mode`, `measured_at`, `source`를 다시 확인하지 않습니다. 따라서 운영
활성화 전에는 `manual_datum`과 측량 근거가 실제로 설정됐는지 별도 검증해야
합니다.

이 경로에서 Coordinator가 말하는 fresh Twist와 Global Odometry는 receipt/stamp
age가 각각 0.20초 이하라는 뜻입니다. 미래 stamp에는 Status Manager의 소스별
0.05초가 아니라 Coordinator의 공통 `candidate_max_future_stamp_sec: 0.10`이
적용됩니다.

이동 중 후보는 정지 후보 카운트에 포함하지 않습니다. Global reset 요청은 최신
Global yaw·Z와 나머지 covariance를 복사하고 GPS x/y와 XY covariance만
교체합니다. reset 후 자동 확인은 Global 결과의 timestamp와 XY 거리만 검사하므로
yaw·Z·나머지 covariance 보존 결과까지 검증하는 것은 아닙니다.

`set_pose_service_wait_timeout_sec: 0.30`은 서비스 존재 대기 한계일 뿐 ROS1 동기
`ServiceClient::call()` 자체의 hard timeout은 아닙니다. 동기 call이 오래 막히면
같은 프로세스의 Manager·Coordinator·Supervisor callback과 heartbeat도 함께
지연됩니다. Global 확인 1.0초 예산도 candidate callback 시작 시각부터 계산되어
서비스 대기·호출 시간이 포함됩니다. 별도 Output Gate는 마지막 `valid`가 stale해지면
최종 Odometry를 차단하지만 그동안 새 공개 상태가 즉시 갱신된다는 보장은 없습니다.

## 현재 기본값에서의 결과

- LiDAR localization: `false`
- GPS-only automatic reset: `false`
- GPS datum: `first_fix`, `measured: false`
- 실제 `maps/map.yaml`: 없음
- `/dev/mando_gps` 별칭과 `ublox_gps`: 현재 호스트에서 없음
- 마지막 실장치 확인 GPS: NMEA 통신은 됐지만 no-fix

따라서 현재 기본 실행은 장기 단절 후 임의 위치 점프를 하지 않습니다. 단기
gate로 안전하게 복귀하지 못하면 복구 상태를 유지하고 최종 위치를 차단합니다.

## 재연결 확인 순서

```text
장치 identity/alias
→ raw serial 통신
→ GPS driver node
→ NavSatFix status·timestamp·frame·covariance·rate
→ quality candidate / gate pose
→ /molit/localization/recovery/state
→ /molit/localization/state + /valid
→ 최종 /molit/localization/odometry
```

장치 파일이 다시 생겼거나 GPS callback이 재개된 것만으로 localization-ready라고
판단하지 않습니다. 실제 지도 사용 시에는 GPS `manual_datum`의 map 원점·yaw와
LiDAR OccupancyGrid가 동일한 `map` 좌표계를 표현하는지도 별도로 확인해야 합니다.
