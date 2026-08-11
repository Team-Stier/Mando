#!/usr/bin/env python3
"""localhost에 안전한 기본값으로 주행 전 map editor를 제공한다."""

import argparse
from pathlib import Path
import re
import signal
import sys

from stier_slam_core.editor_server import create_editor_server


_OPTIONS_WITH_VALUES = {
    "--course-dir", "--workspace-id", "--web-root", "--host", "--port",
    "--unsafe-allow-nonlocal",
}
_ROS_REMAP_PATTERN = re.compile(r"^([~/A-Za-z]|_|__)[\w/]*:=.*$")


def _default_web_root(script_path=None):
    """source checkout 또는 Catkin install prefix에서 web asset 경로를 찾는다."""
    script = Path(__file__ if script_path is None else script_path).resolve()
    candidates = (
        script.parents[1] / "web",
        script.parents[2] / "share" / "stier_slam_core" / "web",
    )
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise ValueError("unable to find stier_slam_core web assets")


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course-dir", required=True)
    parser.add_argument("--workspace-id", required=True)
    parser.add_argument(
        "--web-root",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--unsafe-allow-nonlocal", choices=("false", "true"), default="false"
    )
    return parser


def _without_ros_remapping_args(arguments):
    """일반 option 값의 ':='는 보존하면서 ROS remap argument만 제거한다."""
    filtered = []
    preserve_next = False
    after_separator = False
    for argument in arguments:
        if after_separator:
            filtered.append(argument)
            continue
        if preserve_next:
            filtered.append(argument)
            preserve_next = False
            continue
        if argument == "--":
            filtered.append(argument)
            after_separator = True
            continue
        if argument in _OPTIONS_WITH_VALUES:
            filtered.append(argument)
            preserve_next = True
            continue
        if _ROS_REMAP_PATTERN.fullmatch(argument):
            continue
        filtered.append(argument)
    return filtered


def main(argv=None):
    raw_arguments = sys.argv[1:] if argv is None else list(argv)
    arguments = _parser().parse_args(_without_ros_remapping_args(raw_arguments))
    try:
        web_root = (
            Path(arguments.web_root)
            if arguments.web_root is not None
            else _default_web_root()
        )
        server = create_editor_server(
            arguments.course_dir,
            arguments.workspace_id,
            web_root,
            host=arguments.host,
            port=arguments.port,
            unsafe_allow_nonlocal=arguments.unsafe_allow_nonlocal == "true",
        )
    except (OSError, TypeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2

    def stop_server(signum, frame):
        del signum, frame
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, stop_server)
    bound_host, bound_port = server.server_address[:2]
    display_host = "[{}]".format(bound_host) if ":" in bound_host else bound_host
    print(
        "Stier map editor: http://{}:{}/".format(display_host, bound_port),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
