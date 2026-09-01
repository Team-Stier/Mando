#!/usr/bin/env bash
set -euo pipefail

# Mermaid 원본과 생성물을 패키지 위치 기준으로 찾으므로 어느 디렉터리에서 실행해도 된다.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SOURCE_FILE="${PACKAGE_DIR}/docs/localization_architecture.mmd"
DETAILED_SOURCE_FILE="${PACKAGE_DIR}/docs/localization_architecture_detailed.mmd"
RELOCALIZATION_SOURCE_FILE="${PACKAGE_DIR}/docs/relocalization_manager_architecture.mmd"
RELOCALIZATION_FLOW_SOURCE_FILE="${PACKAGE_DIR}/docs/relocalization_manager_flow.mmd"
OUTPUT_DIR="${PACKAGE_DIR}/docs/images"
SVG_FILE="${OUTPUT_DIR}/localization_architecture.svg"
PNG_FILE="${OUTPUT_DIR}/localization_architecture.png"
DETAILED_SVG_FILE="${OUTPUT_DIR}/localization_architecture_detailed.svg"
DETAILED_PNG_FILE="${OUTPUT_DIR}/localization_architecture_detailed.png"
RELOCALIZATION_SVG_FILE="${OUTPUT_DIR}/relocalization_manager_architecture.svg"
RELOCALIZATION_PNG_FILE="${OUTPUT_DIR}/relocalization_manager_architecture.png"
RELOCALIZATION_FLOW_SVG_FILE="${OUTPUT_DIR}/relocalization_manager_flow.svg"
RELOCALIZATION_FLOW_PNG_FILE="${OUTPUT_DIR}/relocalization_manager_flow.png"
MERMAID_VERSION="11.16.0"

required_nodes=(
  motion_sensors
  absolute_sources
  motion_adapter
  local_ekf
  absolute_correction
  global_ekf
  diagnostics
  recovery_coordinator
  status_manager
  supervisor
  output_gate
  visualization
  tf_contract
  rviz
)

required_tokens=(
  "SerialFeedBack"
  "Serial alive·속도·ADC·delta 검사"
  "Twist.linear.x"
  "Local EKF"
  "Global EKF"
  "odom → base_link"
  "map → odom"
  "tf_broadcast: false"
  "LocalizationStatusManager"
  "RelocalizationCoordinator"
  "LocalizationSupervisor"
  "LocalizationOutputGate"
  "GpsGateReanchor"
  "evaluated status / state / valid"
  "GPS-only 자동 reset 기본 false"
  "/molit/localization/odometry"
)

detailed_required_nodes=(
  gps_gate
  lidar_gate
  recovery
  global
  manager
  supervisor
  recovery_states
  gate
)

detailed_required_tokens=(
  "quality candidate"
  "/molit/localization/gps/map_pose"
  "/internal/gps/reanchor_pose"
  "/internal/gps/reanchor_accepted"
  "transaction_id + stamp + XY 일치"
  "/internal/ekf/global_set_pose"
  "post-reset 증가 stamp 결과 3회"
  "evaluated status / state / valid"
  "recovery state / active"
  "누락·stale → FAULT"
)

relocalization_required_nodes=(
  ingress
  activate
  lidar_mismatch
  lidar_ready
  gps_blocked
  set_pose
  reanchor
  finish
  timer
  supervisor
  output_gate
)

relocalization_required_tokens=(
  "별도 relocalization 노드가 아님"
  "quality candidate"
  "마지막 gate 승인 후 > 2.0 s"
  "GPS-only로 우회하지 않음"
  "automatic_reset_enabled=true"
  "/mando_localization/internal/ekf/global_set_pose"
  "서로 다른 증가 stamp 3회 연속"
  "GpsGateReanchor transaction_id + target pose"
  "transaction_id 정확히 일치"
  "active=true → RELOCALIZING, valid=false"
  "SetPose 동기 call hard timeout 없음"
)

relocalization_flow_required_nodes=(
  inputs
  coordinator
  outage
  normal
  strategy
  lidar_recovery
  gps_recovery
  reanchor
  approved_pose
  global_ekf
  supervisor
  output_gate
  final_odom
)

relocalization_flow_required_tokens=(
  "간단 아키텍처 Flow"
  "정상 경로"
  "장기 단절"
  "LiDAR 보조 복구"
  "GPS-only 복구"
  "기본 비활성"
  "transaction ACK"
  "RELOCALIZING"
  "/molit/localization/odometry"
)

if [[ ! -s "${SOURCE_FILE}" ]]; then
  echo "오류: Mermaid 원본이 없거나 비어 있습니다: ${SOURCE_FILE}" >&2
  exit 1
fi

if [[ ! -s "${DETAILED_SOURCE_FILE}" ]]; then
  echo "오류: 상세 Mermaid 원본이 없거나 비어 있습니다: ${DETAILED_SOURCE_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_SOURCE_FILE}" ]]; then
  echo "오류: 재정합 Manager Mermaid 원본이 없거나 비어 있습니다: ${RELOCALIZATION_SOURCE_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_FLOW_SOURCE_FILE}" ]]; then
  echo "오류: 재정합 Manager 간단 Flow Mermaid 원본이 없거나 비어 있습니다: ${RELOCALIZATION_FLOW_SOURCE_FILE}" >&2
  exit 1
fi

for node_id in "${required_nodes[@]}"; do
  if ! grep -Eq "^[[:space:]]*${node_id}\\[" "${SOURCE_FILE}"; then
    echo "오류: 아키텍처에 필수 노드가 없습니다: ${node_id}" >&2
    exit 1
  fi
done

for token in "${required_tokens[@]}"; do
  if ! grep -Fq "${token}" "${SOURCE_FILE}"; then
    echo "오류: 아키텍처에 필수 계약 표기가 없습니다: ${token}" >&2
    exit 1
  fi
done

for node_id in "${detailed_required_nodes[@]}"; do
  if ! grep -Eq "^[[:space:]]*${node_id}\\[" "${DETAILED_SOURCE_FILE}"; then
    echo "오류: 상세 아키텍처에 필수 노드가 없습니다: ${node_id}" >&2
    exit 1
  fi
done

for token in "${detailed_required_tokens[@]}"; do
  if ! grep -Fq "${token}" "${DETAILED_SOURCE_FILE}"; then
    echo "오류: 상세 아키텍처에 필수 계약 표기가 없습니다: ${token}" >&2
    exit 1
  fi
done


for node_id in "${relocalization_required_nodes[@]}"; do
  if ! grep -Eq "^[[:space:]]*${node_id}\\[" "${RELOCALIZATION_SOURCE_FILE}"; then
    echo "오류: 재정합 Manager 아키텍처에 필수 노드가 없습니다: ${node_id}" >&2
    exit 1
  fi
done

for token in "${relocalization_required_tokens[@]}"; do
  if ! grep -Fq "${token}" "${RELOCALIZATION_SOURCE_FILE}"; then
    echo "오류: 재정합 Manager 아키텍처에 필수 계약 표기가 없습니다: ${token}" >&2
    exit 1
  fi
done

for node_id in "${relocalization_flow_required_nodes[@]}"; do
  if ! grep -Eq "^[[:space:]]*${node_id}(\\[|\\{)" "${RELOCALIZATION_FLOW_SOURCE_FILE}"; then
    echo "오류: 재정합 Manager 간단 Flow에 필수 노드가 없습니다: ${node_id}" >&2
    exit 1
  fi
done

for token in "${relocalization_flow_required_tokens[@]}"; do
  if ! grep -Fq "${token}" "${RELOCALIZATION_FLOW_SOURCE_FILE}"; then
    echo "오류: 재정합 Manager 간단 Flow에 필수 계약 표기가 없습니다: ${token}" >&2
    exit 1
  fi
done

mkdir -p "${OUTPUT_DIR}"

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${SOURCE_FILE}" \
  -o "${SVG_FILE}" \
  -b transparent

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${SOURCE_FILE}" \
  -o "${PNG_FILE}" \
  -s 4 \
  -b white

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${DETAILED_SOURCE_FILE}" \
  -o "${DETAILED_SVG_FILE}" \
  -b transparent

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${DETAILED_SOURCE_FILE}" \
  -o "${DETAILED_PNG_FILE}" \
  -s 3 \
  -b white

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${RELOCALIZATION_SOURCE_FILE}" \
  -o "${RELOCALIZATION_SVG_FILE}" \
  -b transparent

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${RELOCALIZATION_SOURCE_FILE}" \
  -o "${RELOCALIZATION_PNG_FILE}" \
  -s 5 \
  -b white

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${RELOCALIZATION_FLOW_SOURCE_FILE}" \
  -o "${RELOCALIZATION_FLOW_SVG_FILE}" \
  -b transparent

npx --yes "@mermaid-js/mermaid-cli@${MERMAID_VERSION}" \
  -i "${RELOCALIZATION_FLOW_SOURCE_FILE}" \
  -o "${RELOCALIZATION_FLOW_PNG_FILE}" \
  -s 5 \
  -b white

if [[ ! -s "${SVG_FILE}" ]] || ! grep -q '<svg' "${SVG_FILE}"; then
  echo "오류: 생성된 SVG가 유효하지 않습니다: ${SVG_FILE}" >&2
  exit 1
fi

# connector는 수평·수직 직선만 허용한다. Q/C 명령은 곡선 구간을 의미한다.
if grep -Eo 'd="[^"]+"' "${SVG_FILE}" | grep -Eq '[QC]'; then
  echo "오류: 생성된 SVG 연결선에 곡선 구간이 포함되어 있습니다: ${SVG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${PNG_FILE}" ]] || ! file "${PNG_FILE}" | grep -q 'PNG image data'; then
  echo "오류: 생성된 PNG가 유효하지 않습니다: ${PNG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${DETAILED_SVG_FILE}" ]] || ! grep -q '<svg' "${DETAILED_SVG_FILE}"; then
  echo "오류: 생성된 상세 SVG가 유효하지 않습니다: ${DETAILED_SVG_FILE}" >&2
  exit 1
fi

if grep -Eo 'd="[^"]+"' "${DETAILED_SVG_FILE}" | grep -Eq '[QC]'; then
  echo "오류: 생성된 상세 SVG 연결선에 곡선 구간이 포함되어 있습니다: ${DETAILED_SVG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${DETAILED_PNG_FILE}" ]] || ! file "${DETAILED_PNG_FILE}" | grep -q 'PNG image data'; then
  echo "오류: 생성된 상세 PNG가 유효하지 않습니다: ${DETAILED_PNG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_SVG_FILE}" ]] || ! grep -q '<svg' "${RELOCALIZATION_SVG_FILE}"; then
  echo "오류: 생성된 재정합 Manager SVG가 유효하지 않습니다: ${RELOCALIZATION_SVG_FILE}" >&2
  exit 1
fi

if grep -Eo 'd="[^"]+"' "${RELOCALIZATION_SVG_FILE}" | grep -Eq '[QC]'; then
  echo "오류: 생성된 재정합 Manager SVG 연결선에 곡선 구간이 포함되어 있습니다: ${RELOCALIZATION_SVG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_PNG_FILE}" ]] || ! file "${RELOCALIZATION_PNG_FILE}" | grep -q 'PNG image data'; then
  echo "오류: 생성된 재정합 Manager PNG가 유효하지 않습니다: ${RELOCALIZATION_PNG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_FLOW_SVG_FILE}" ]] || ! grep -q '<svg' "${RELOCALIZATION_FLOW_SVG_FILE}"; then
  echo "오류: 생성된 재정합 Manager 간단 Flow SVG가 유효하지 않습니다: ${RELOCALIZATION_FLOW_SVG_FILE}" >&2
  exit 1
fi

if grep -Eo 'd="[^"]+"' "${RELOCALIZATION_FLOW_SVG_FILE}" | grep -Eq '[QC]'; then
  echo "오류: 생성된 재정합 Manager 간단 Flow SVG 연결선에 곡선 구간이 포함되어 있습니다: ${RELOCALIZATION_FLOW_SVG_FILE}" >&2
  exit 1
fi

if [[ ! -s "${RELOCALIZATION_FLOW_PNG_FILE}" ]] || ! file "${RELOCALIZATION_FLOW_PNG_FILE}" | grep -q 'PNG image data'; then
  echo "오류: 생성된 재정합 Manager 간단 Flow PNG가 유효하지 않습니다: ${RELOCALIZATION_FLOW_PNG_FILE}" >&2
  exit 1
fi

echo "아키텍처 이미지 생성 완료"
echo "  SVG: ${SVG_FILE}"
echo "  PNG: ${PNG_FILE}"
echo "상세 아키텍처 이미지 생성 완료"
echo "  SVG: ${DETAILED_SVG_FILE}"
echo "  PNG: ${DETAILED_PNG_FILE}"
echo "재정합 Manager 상세 아키텍처 이미지 생성 완료"
echo "  SVG: ${RELOCALIZATION_SVG_FILE}"
echo "  PNG: ${RELOCALIZATION_PNG_FILE}"
echo "재정합 Manager 간단 Flow 이미지 생성 완료"
echo "  SVG: ${RELOCALIZATION_FLOW_SVG_FILE}"
echo "  PNG: ${RELOCALIZATION_FLOW_PNG_FILE}"
