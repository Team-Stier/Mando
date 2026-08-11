# Mando SLAM Package Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 검증된 SLAM 프로젝트를 `Team-Stier/Mando`의 `Paik/Slam_pkg` 브랜치에 `Slam_pkg/` 폴더로 가져온다.

**Architecture:** 기존 SLAM checkout은 그대로 보존하고 Mando `main`에서 별도 checkout을 만든다. 재현 가능한 소스만 `Slam_pkg/`에 복사하고, 해당 위치에서 catkin 빌드와 전체 테스트를 수행한다.

**Tech Stack:** Git, Bash, ROS1 Noetic, catkin, Python 3.8

## Global Constraints

- Mando `main`과 기존 `arduino/`는 수정하지 않는다.
- 대상 브랜치는 정확히 `Paik/Slam_pkg`다.
- 대상 remote는 `origin=https://github.com/Team-Stier/Mando.git`다.
- `.git`, `.ruff_cache`, `.superpowers`, `build`, `devel`, `install`, `__pycache__`, `*.pyc`는 복사하지 않는다.
- 기존 SLAM checkout 파일은 이동하거나 삭제하지 않는다.
- push와 PR 생성은 이 계획의 범위 밖이다.

---

### Task 1: Mando 작업 checkout 준비

**Files:**
- Preserve: `/home/stier/slam/.worktrees/rtabmap-slam-implementation`
- Create: `/home/stier/Mando`

**Interfaces:**
- Consumes: `https://github.com/Team-Stier/Mando.git`, remote branch `main`
- Produces: clean local branch `Paik/Slam_pkg` tracking Mando history

- [ ] `git clone https://github.com/Team-Stier/Mando.git /home/stier/Mando`로 별도 checkout을 만든다.
- [ ] `/home/stier/Mando`에서 `git switch -c Paik/Slam_pkg origin/main`을 실행한다.
- [ ] `git remote get-url origin`, `git branch --show-current`, `git status --short`로 remote, branch, clean state를 검증한다.

### Task 2: SLAM 프로젝트를 `Slam_pkg/`로 복사

**Files:**
- Create: `/home/stier/Mando/Slam_pkg/.catkin_workspace`
- Create: `/home/stier/Mando/Slam_pkg/.gitignore`
- Create: `/home/stier/Mando/Slam_pkg/README.md`
- Create: `/home/stier/Mando/Slam_pkg/src/**`
- Create: `/home/stier/Mando/Slam_pkg/scripts/**`
- Create: `/home/stier/Mando/Slam_pkg/docs/**`

**Interfaces:**
- Consumes: committed SLAM tree plus `docs/system_architecture.mmd`, `.png`, `.svg`
- Produces: standalone catkin workspace rooted at `/home/stier/Mando/Slam_pkg`

- [ ] Git tracked 파일 목록을 기준으로 `.catkin_workspace`, `.gitignore`, `README.md`, `src/`, `scripts/`, `docs/`를 복사한다.
- [ ] 미추적 아키텍처 파일 세 개를 `Slam_pkg/docs/`에 복사한다.
- [ ] `find Slam_pkg`로 생성물과 cache가 없고 `arduino/`가 변경되지 않았는지 검사한다.

### Task 3: 빌드·테스트 및 로컬 커밋

**Files:**
- Add: `/home/stier/Mando/Slam_pkg/**`

**Interfaces:**
- Consumes: standalone `Slam_pkg` catkin workspace
- Produces: verified local commit on `Paik/Slam_pkg`

- [ ] `source /opt/ros/noetic/setup.bash && PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`를 `Slam_pkg/`에서 실행한다.
- [ ] `PYTHONNOUSERSITE=1 catkin_make run_tests -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3`를 실행한다.
- [ ] `catkin_test_results build/test_results`에서 error와 failure가 모두 0인지 확인한다.
- [ ] 생성된 `build/`, `devel/`, `install/`이 Git 대상에서 제외되는지 확인한다.
- [ ] `git add Slam_pkg` 후 `git diff --cached --check`를 실행한다.
- [ ] `git commit -m "feat: SLAM 패키지 추가"`로 로컬 커밋한다.
- [ ] 최종 branch, remote, commit, status를 보고하고 push는 수행하지 않는다.
