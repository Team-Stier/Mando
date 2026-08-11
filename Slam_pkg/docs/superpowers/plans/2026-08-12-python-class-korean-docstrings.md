# Python 운영 클래스 한국어 Docstring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SLAM 운영 Python 클래스의 역할과 동작 방식을 클래스 선언부에서 바로 이해할 수 있게 한다.

**Architecture:** 실행 노드와 핵심 로직의 주요 클래스 docstring만 한국어 `역할` 및 `동작 방식` 형식으로 보강한다. 실행문, 인터페이스, 설정값은 변경하지 않고 기존 테스트로 동작 보존을 확인한다.

**Tech Stack:** Python 3.8, ROS1 Noetic, unittest, Catkin

## Global Constraints

- ROS 토픽, 메시지 타입, 클래스·함수·변수 이름은 영어 표기를 유지한다.
- 실행 로직, ROS 인터페이스, 설정값 및 예외 처리를 변경하지 않는다.
- 테스트 클래스와 단순 예외 클래스는 제외한다.
- 사용자 미추적 `docs/system_architecture.*` 파일은 수정하거나 커밋하지 않는다.

---

### Task 1: 센서 및 위치 추정 클래스 설명

**Files:**
- Modify: `src/stier_slam_core/src/stier_slam_core/ackermann.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/scan_leveling.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/rtk_gate.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/freshness.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/live_obstacles.py`
- Modify: `src/stier_slam_core/scripts/vehicle_odometry_node.py`
- Modify: `src/stier_slam_core/scripts/scan_leveler_node.py`
- Modify: `src/stier_slam_core/scripts/rtk_gate_node.py`
- Modify: `src/stier_slam_core/scripts/sensor_health_node.py`
- Modify: `src/stier_slam_core/scripts/live_obstacle_node.py`

**Interfaces:**
- Consumes: 기존 클래스 선언과 기존 영어 docstring
- Produces: `역할`과 `동작 방식`을 포함하는 한국어 class docstring

- [x] **Step 1:** 공개 데이터 클래스, 처리 클래스 및 ROS 실행 노드의 기존 책임과 입력·처리·출력을 읽는다.
- [x] **Step 2:** 각 주요 클래스 선언 바로 아래 docstring을 `역할`과 `동작 방식` 형식으로 작성한다.
- [x] **Step 3:** `git diff --check`와 diff 검토로 실행문이 바뀌지 않았는지 확인한다.

### Task 2: 지도 편집·배포·runtime 클래스 설명

**Files:**
- Modify: `src/stier_slam_core/src/stier_slam_core/grid_map.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/map_overlay.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/map_release.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/runtime_database.py`
- Modify: `src/stier_slam_core/src/stier_slam_core/editor_server.py`
- Modify: `src/stier_slam_core/scripts/map_editor_server.py`
- Modify: `src/stier_slam_core/scripts/map_release_cli.py`
- Modify: `src/stier_slam_core/scripts/rtabmap_runtime_guard.py`

**Interfaces:**
- Consumes: 기존 map workspace, release, HTTP 및 runtime guard 계약
- Produces: 주요 운영 클래스의 한국어 역할·동작 docstring

- [x] **Step 1:** 지도 파일 로딩, overlay 편집, release 게시, runtime database 보호 흐름을 읽는다.
- [x] **Step 2:** 학습에 의미 있는 주요 클래스에만 간결한 한국어 docstring을 작성한다.
- [x] **Step 3:** private helper라도 핵심 안전 계약을 소유하면 포함하고 단순 예외 클래스는 제외한다.

### Task 3: 문법 및 회귀 검증

**Files:**
- Verify: `src/stier_slam_core/src/stier_slam_core/*.py`
- Verify: `src/stier_slam_core/scripts/*.py`
- Test: `src/stier_slam_core/test/test_*.py`

**Interfaces:**
- Consumes: Task 1과 Task 2의 docstring 변경
- Produces: 문법 정상 및 기존 테스트 통과 증거

- [x] **Step 1:** `/usr/bin/python3 -m compileall -q src/stier_slam_core/src src/stier_slam_core/scripts`를 실행한다.
- [x] **Step 2:** `PYTHONPATH=src/stier_slam_core/src /usr/bin/python3 -m unittest discover -s src/stier_slam_core/test -p 'test_*.py'`를 실행한다.
- [x] **Step 3:** `git diff --check`, `git diff --stat`, `git status --short`로 변경 범위와 사용자 미추적 파일 보존을 확인한다.
