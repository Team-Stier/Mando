# Central ROS Topic Configuration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SLAM 토픽 계약을 `topics.yaml`에 모으고 자체 Python node가 해당 ROS parameter를 사용하게 한다.

**Architecture:** global `/stier_slam/topics` parameter tree를 launch에서 load한다. 자체 node는 공통 parameter 경로를 읽으며, 외부 RTAB-Map/EKF/record 설정은 contract test에서 중앙 YAML과 일치하는지 검증한다.

**Tech Stack:** ROS1 Noetic, roslaunch XML, YAML, Python 3.8, unittest

## Global Constraints

- 기존 기본 토픽명과 메시지 type을 유지한다.
- parameter 누락은 node 시작 실패로 처리한다.
- 사용자 미추적 architecture 파일은 보존한다.

---

### Task 1: 중앙 토픽 계약 RED test

**Files:**
- Create: `src/stier_slam_bringup/config/topics.yaml`
- Modify: `src/stier_slam_bringup/test/test_launch_contracts.py`
- Modify: `src/stier_slam_core/test/test_vehicle_odometry_node.py`
- Modify: `src/stier_slam_core/test/test_scan_leveling.py`
- Modify: `src/stier_slam_core/test/test_rtk_gate.py`
- Modify: `src/stier_slam_core/test/test_freshness.py`
- Modify: `src/stier_slam_core/test/test_live_obstacles.py`

- [x] 중앙 YAML 존재, schema, 기본값 및 node topic override를 요구하는 테스트를 작성한다.
- [x] 테스트를 실행해 YAML과 parameter 사용이 없어 실패하는지 확인한다.

### Task 2: 자체 node와 launch GREEN 구현

**Files:**
- Modify: `src/stier_slam_core/scripts/vehicle_odometry_node.py`
- Modify: `src/stier_slam_core/scripts/scan_leveler_node.py`
- Modify: `src/stier_slam_core/scripts/rtk_gate_node.py`
- Modify: `src/stier_slam_core/scripts/sensor_health_node.py`
- Modify: `src/stier_slam_core/scripts/live_obstacle_node.py`
- Modify: `src/stier_slam_bringup/launch/mapping.launch`
- Modify: `src/stier_slam_bringup/launch/localization.launch`
- Modify: `src/stier_slam_bringup/launch/synthetic_demo.launch`

- [x] `/stier_slam/topics/...` parameter를 읽어 publisher/subscriber를 생성한다.
- [x] 세 launch에 `topics.yaml`을 한 번 load한다.
- [x] focused unit test를 실행해 GREEN을 확인한다.

### Task 3: 외부 node 계약과 전체 검증

**Files:**
- Modify: `src/stier_slam_bringup/test/test_launch_contracts.py`
- Verify: `src/stier_slam_bringup/config/ekf_local.yaml`
- Verify: `src/stier_slam_bringup/launch/record.launch`

- [x] RTAB-Map remap, EKF input과 record topics가 중앙 YAML과 같은지 검사한다.
- [x] `compileall`, core unit test와 launch contract test를 실행한다.
- [x] diff와 status를 확인하고 사용자 미추적 파일을 보존한다.
