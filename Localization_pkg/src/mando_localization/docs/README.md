# mando_localization 문서

이 문서는 현재 checkout의 C++·YAML·launch·RViz 계약을 기준으로 작성했습니다.

| 문서 | 내용 |
|---|---|
| [architecture.md](architecture.md) | 공개/내부 토픽 어댑터, Supervisor 내부 구성, GPS 단일 공개 경계, Local·Global EKF, AMCL과 안전 출력 흐름 |
| [configuration.md](configuration.md) | YAML별 실제 키, 단위, 제약, 변경 영향과 launch 인자 |
| [tf_frames.md](tf_frames.md) | 동적 TF 소유권 검증, 정적 TF 측정·활성화, 런타임 확인 |
| [gps_quality_and_recovery.md](gps_quality_and_recovery.md) | GPS status 0/1/2, covariance 출처, 10 m gate, 짧은/긴 단절과 LiDAR·GPS-only 복구 |
| [status_and_recovery.md](status_and_recovery.md) | Supervisor 중재 우선순위, `GpsGateReanchor` transaction/ACK, recovery 상태, DR 예산과 Output Gate 조건 |
| [localization_architecture.mmd](localization_architecture.mmd) | 아래 SVG·PNG의 Mermaid 원본 |
| [localization_architecture_detailed.mmd](localization_architecture_detailed.mmd) | 토픽·메시지·알고리즘 수치가 포함된 상세 Mermaid 원본 |
| [relocalization_manager_architecture.mmd](relocalization_manager_architecture.mmd) | Relocalization Manager만 분리한 입력·상태·LiDAR/GPS-only 분기·ACK 상세 Mermaid 원본 |
| [relocalization_manager_flow.mmd](relocalization_manager_flow.mmd) | 세부 임곗값 없이 Relocalization Manager 처리 흐름만 표시한 Mermaid 원본 |

## 전체 아키텍처

![위치추정 아키텍처](images/localization_architecture.svg)

## 전체 상세 아키텍처

아래 그림은 현재 YAML 기본값을 기준으로 공개·내부 토픽, 메시지 타입, EKF 관측 상태,
품질 게이트 수식과 임곗값, 상태 timeout 및 fail-closed 출력 조건까지 표시합니다.
숫자는 실차 성능 측정값이 아니라 현재 설정 계약이며, 설정 변경 시 그림도 함께 갱신해야 합니다.

![위치추정 상세 아키텍처](images/localization_architecture_detailed.svg)

## Relocalization Manager 간단 아키텍처 Flow

세부 토픽·임곗값·내부 상태를 제외하고 정상 relay와 두 복구 분기, ACK,
Global EKF 및 fail-closed 출력 연결만 표시합니다.

![Relocalization Manager 간단 아키텍처 Flow](images/relocalization_manager_flow.svg)

## Relocalization Manager 단독 상세 아키텍처

이 그림은 별도 ROS 노드가 아닌 `localization_supervisor` 프로세스 내부의
`RelocalizationCoordinator`와 순수 결정 로직 `RelocalizationPolicy`만 분리해
보여줍니다. 정상 GPS gate relay, 장기 단절 진입, LiDAR 보조 경로, 기본 차단된
GPS-only reset, Global 확인, transaction ACK와 Supervisor fail-closed 연결이
현재 YAML 기본값 기준으로 포함돼 있습니다.

![Relocalization Manager 상세 아키텍처](images/relocalization_manager_architecture.svg)

그림의 단계 번호는 노드가 데이터를 처리하는 순서입니다. 실선은 위치 계산 흐름,
점선은 계산에 관여하지 않는 상태 판단 흐름이며, 연결선은 수평·수직 직교선으로
렌더링합니다. 세부 토픽, 장치 설정과 실패 정책은 아래 문서에서 확인할 수 있습니다.

- 실선: 위치 계산·relay·시각화 데이터 흐름
- 점선: 상태 평가·재정합 command/ack와 Supervisor 중재 신호
- TF 간선: Local/Global EKF와 검증된 정적 TF의 유일 소유 관계

SVG는 확대해도 선명한 Markdown 열람용이고, PNG는 다이어그램별 Mermaid 기본 크기의 3–5배 배율로 생성하는 발표·인쇄용입니다. Relocalization Manager의 간단·상세 PNG는 모두 5배 배율로 생성합니다. 원본을 수정하면 다음 명령으로 모두 함께 갱신합니다.

```bash
cd /home/stier/Mando_1-5/localization/src/mando_localization
./scripts/render_architecture.sh
```
