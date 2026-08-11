# Wheel Encoder Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 표준 JointState wheel encoder를 기존 vehicle-speed/Ackermann/EKF 경로에 연결한다.

**Architecture:** ROS 독립 계산기와 얇은 ROS adapter를 분리한다. mapping/localization에서만 adapter를 실행하며 synthetic fixture는 기존 속도 publisher를 유지한다.

**Tech Stack:** Python 3.8, ROS1 Noetic, sensor_msgs, geometry_msgs, unittest

## Global Constraints

- source timestamp를 보존한다.
- 실차 calibration 전에는 fail closed한다.
- 기존 `/slam/input/vehicle_speed` 소비자와 메시지 type을 유지한다.

---

### Task 1: Encoder 계산 RED/GREEN

**Files:**
- Create: `src/stier_slam_core/src/stier_slam_core/wheel_encoder.py`
- Create: `src/stier_slam_core/test/test_wheel_encoder.py`

- [x] velocity 및 position 차분 계약의 실패 테스트를 작성하고 RED를 확인한다.
- [x] 최소 계산기를 구현하고 focused test GREEN을 확인한다.

### Task 2: ROS adapter RED/GREEN

**Files:**
- Create: `src/stier_slam_core/scripts/wheel_encoder_adapter_node.py`
- Create: `src/stier_slam_core/test/test_wheel_encoder_adapter_node.py`
- Create: `src/stier_slam_bringup/config/encoder.yaml`
- Modify: `src/stier_slam_bringup/config/topics.yaml`

- [x] JointState 구독과 TwistWithCovarianceStamped 출력을 요구하는 실패 테스트를 작성한다.
- [x] source stamp, frame과 covariance를 보존하는 최소 wrapper를 구현한다.

### Task 3: Launch와 기록 연결

**Files:**
- Modify: `src/stier_slam_core/CMakeLists.txt`
- Modify: `src/stier_slam_bringup/launch/mapping.launch`
- Modify: `src/stier_slam_bringup/launch/localization.launch`
- Modify: `src/stier_slam_bringup/launch/record.launch`
- Modify: `src/stier_slam_bringup/test/test_launch_contracts.py`
- Modify: `README.md`

- [x] launch/record 계약 실패 테스트를 작성하고 RED를 확인한다.
- [x] 실제 launch에 required adapter를 추가하고 synthetic fixture에는 추가하지 않는다.
- [x] compileall, 전체 unit test와 launch contract를 실행한다.
