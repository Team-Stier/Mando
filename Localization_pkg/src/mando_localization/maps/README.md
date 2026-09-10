# LiDAR 지도 파일 위치

실측 지도를 만든 뒤 이 디렉터리에 `map.yaml`과 해당 이미지(`map.pgm` 또는
`map.png`)를 넣습니다. `map.yaml.example`을 복사해 해상도와 원점을 실제 지도에
맞게 수정하십시오.

범용 `bringup.launch`는 LiDAR 위치추정이 기본 활성화되어 있으므로 실행 전에 지도와
`base_link -> laser_link` 실측 TF를 모두 준비해야 합니다. 지도가 아직 없으면
범용 launch에 `enable_lidar_localization:=false`를 지정합니다. `localization`과
`replay.launch`는 이미 LiDAR 위치추정을 끕니다. 예제 파일은 구조
설명용이며 운영 지도처럼 사용하지 않습니다.

LiDAR-assisted GPS 재정합에서는 AMCL pose와 GPS projected pose를 직접
비교합니다. 따라서 OccupancyGrid 원점·yaw와 측량 `manual_datum`의 map 좌표가
동일한 `map` 기준임을 검증해야 합니다. 지도 기준이 확인되지 않은 `first_fix`
GPS와 AMCL pose를 장기 단절 복구의 교차검증 기준으로 사용하지 않습니다.
