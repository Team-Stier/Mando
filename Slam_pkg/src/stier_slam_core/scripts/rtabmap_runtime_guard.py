#!/usr/bin/env python3
"""RTAB-Map node process를 exec하기 전에 변경 가능한 runtime artifact를 준비한다."""

import sys

from stier_slam_core.runtime_database import guard_main


if __name__ == "__main__":
    sys.exit(guard_main())
