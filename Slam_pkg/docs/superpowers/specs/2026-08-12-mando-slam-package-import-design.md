# Mando Slam Package Import Design

## Goal

현재 검증된 SLAM 프로젝트를 `Team-Stier/Mando`의 `main` 이력 위에
`Paik/Slam_pkg` 브랜치로 가져오고, 저장소 루트의 `Slam_pkg/` 아래에 배치한다.

## Repository strategy

- 기존 SLAM checkout과 `feature/rtabmap-slam-implementation` 브랜치는 백업으로 보존한다.
- `Team-Stier/Mando/main`에서 별도 checkout을 만들고 `Paik/Slam_pkg`를 생성한다.
- Mando remote 이름은 `origin`, URL은 `https://github.com/Team-Stier/Mando.git`로 둔다.
- 서로 다른 Git 이력을 강제로 병합하거나 Mando `main`을 덮어쓰지 않는다.

## Import scope

`Slam_pkg/`에는 다음 재현 가능한 프로젝트 파일을 넣는다.

- `.catkin_workspace`, `.gitignore`, `README.md`
- `src/`, `scripts/`, `docs/`
- 사용자가 작성한 `docs/system_architecture.mmd`, `.png`, `.svg`

다음 로컬 생성물과 작업 도구 상태는 넣지 않는다.

- `.git/`, `.ruff_cache/`, `.superpowers/`
- `build/`, `devel/`, `install/`
- `__pycache__/`, `*.pyc`

## Validation

- Mando checkout의 현재 브랜치와 remote가 정확한지 확인한다.
- `Slam_pkg/` 파일 목록과 제외 대상 부재를 검사한다.
- `/usr/bin/python3`와 ROS Noetic으로 `Slam_pkg` catkin build 및 전체 테스트를 실행한다.
- 검증 후 작은 한국어 커밋을 만들되 push나 PR은 별도 명시 없이는 수행하지 않는다.

## Safety

- 기존 Mando `arduino/` 파일은 수정하지 않는다.
- 기존 SLAM checkout의 파일을 이동하거나 삭제하지 않고 복사한다.
- Mando 원격 `main`과 다른 원격 브랜치를 변경하지 않는다.
