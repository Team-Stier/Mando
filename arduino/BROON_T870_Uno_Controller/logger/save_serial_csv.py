#!/usr/bin/env python3
"""Save T870 CAN logger serial output as timestamped CSV."""

from __future__ import annotations

import argparse
import datetime as dt
import pathlib
import sys

import serial


def default_output_path() -> pathlib.Path:
    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return pathlib.Path(f"t870_can_{stamp}.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save T870CanCsvLogger output with host wall-clock time."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=pathlib.Path, default=default_output_path())
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    header_written = False

    with serial.Serial(args.port, args.baud, timeout=1) as connection, args.output.open(
        "w", encoding="utf-8", newline=""
    ) as output:
        print(f"logging {args.port} -> {args.output}", file=sys.stderr)
        while True:
            raw = connection.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line, file=sys.stderr)
                continue
            if line.startswith("logger_ms,"):
                if not header_written:
                    output.write("host_time_iso," + line + "\n")
                    output.flush()
                    header_written = True
                continue
            if not header_written:
                print("waiting for CSV header; discarded: " + line, file=sys.stderr)
                continue

            host_time = dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
            output.write(host_time + "," + line + "\n")
            output.flush()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
