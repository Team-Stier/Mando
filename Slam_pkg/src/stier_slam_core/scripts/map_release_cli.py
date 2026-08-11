#!/usr/bin/env python3
"""불변 occupancy map artifact를 다루는 간단한 command-line adapter다."""

import argparse
from datetime import datetime, timezone
import json
import sys

from stier_slam_core.map_release import ReleaseManager


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    init_capture = commands.add_parser("init-capture")
    _course_argument(init_capture)
    init_capture.add_argument("--capture", required=True)
    init_capture.add_argument("--db", required=True)
    init_capture.add_argument("--map-yaml", required=True)

    init_workspace = commands.add_parser("init-workspace")
    _course_argument(init_workspace)
    init_workspace.add_argument("--capture", required=True)
    init_workspace.add_argument("--workspace", required=True)

    validate = commands.add_parser("validate")
    _course_argument(validate)
    validate.add_argument("--workspace", required=True)

    release = commands.add_parser("release")
    _course_argument(release)
    release.add_argument("--workspace", required=True)
    release.add_argument("--release", required=True)

    activate = commands.add_parser("activate")
    _course_argument(activate)
    activate.add_argument("--release", required=True)
    return parser


def _course_argument(parser):
    parser.add_argument("--course", required=True)


def main(argv=None):
    arguments = _parser().parse_args(argv)
    manager = ReleaseManager()
    try:
        if arguments.command == "init-capture":
            path = manager.init_capture(
                arguments.course,
                arguments.capture,
                arguments.db,
                arguments.map_yaml,
                now_iso=_now_iso(),
            )
            result = {"capture_path": str(path)}
        elif arguments.command == "init-workspace":
            path = manager.init_workspace(
                arguments.course, arguments.capture, arguments.workspace
            )
            result = {"workspace_path": str(path)}
        elif arguments.command == "validate":
            result = manager.validate_workspace(arguments.course, arguments.workspace)
        elif arguments.command == "release":
            validation = manager.validate_workspace(arguments.course, arguments.workspace)
            path = manager.create_release(
                arguments.course,
                validation["capture_id"],
                arguments.workspace,
                arguments.release,
                _now_iso(),
            )
            result = {"release_path": str(path), "release_id": arguments.release}
        else:
            path = manager.activate_release(arguments.course, arguments.release)
            result = {"active_manifest": str(path), "release_id": arguments.release}
    except (OSError, TypeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


def _now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


if __name__ == "__main__":
    sys.exit(main())
