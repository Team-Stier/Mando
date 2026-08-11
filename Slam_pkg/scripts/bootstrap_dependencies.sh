#!/usr/bin/env bash
set -euo pipefail

readonly REQUIRED_PACKAGES=(
  build-essential
  cmake
  nodejs
  python3-nose
  python3-setuptools
  python3-yaml
  ros-noetic-catkin
  ros-noetic-map-server
  ros-noetic-robot-localization
  ros-noetic-rosbag
  ros-noetic-rosnode
  ros-noetic-rostest
  ros-noetic-rtabmap-ros
  ros-noetic-rviz
  ros-noetic-tf2-sensor-msgs
)

usage() {
  echo "Usage: $0 {--check|--install}" >&2
}

check_dependencies() {
  local package
  local status
  local missing=0

  for package in "${REQUIRED_PACKAGES[@]}"; do
    if status="$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null)" \
        && [[ "$status" == "install ok installed" ]]; then
      continue
    fi
    echo "Missing package: $package"
    missing=1
  done

  if (( missing )); then
    return 1
  fi
  echo "All required packages are installed."
}

install_dependencies() {
  local package
  echo "Installing packages:"
  for package in "${REQUIRED_PACKAGES[@]}"; do
    echo "  $package"
  done
  sudo apt-get install "${REQUIRED_PACKAGES[@]}"
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

case "$1" in
  --check)
    check_dependencies
    ;;
  --install)
    install_dependencies
    ;;
  *)
    usage
    exit 2
    ;;
esac
