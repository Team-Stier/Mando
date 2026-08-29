#!/usr/bin/env python3
"""Capture T870 CAN logger CSV and run the rule-based diagnosis in one command."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import re
import subprocess
import sys
import time


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
CONTROLLER_DIR = SCRIPT_DIR.parent
ANALYZER = CONTROLLER_DIR / "fault_diagnosis" / "analyze_t870_log.py"
DEFAULT_RUN_ROOT = CONTROLLER_DIR / "runs"
LOGGER_HEADER = (
    "logger_ms,complete,seq,protocol,state,fault,mode,status_flags,"
    "uptime_s,drive_req_pwm,front_pwm,rear_pwm,steer_target_adc,"
    "steer_actual_adc,steer_pwm,speed_kph,encoder_delta,encoder_corrected,"
    "rc_steer_us,rc_throttle_us,rc_aux_us,rc_flags,rc_read_us,tx_dropped,"
    "can_eflg,can_tec,can_rec"
)
LOGGER_COLUMN_COUNT = len(LOGGER_HEADER.split(","))


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z가-힣._-]+", "_", value.strip())
    return cleaned.strip("._-") or "run"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture T870CanCsvLogger serial data and automatically create a "
            "rule-based diagnosis report when capture stops."
        )
    )
    parser.add_argument("--port", required=True, help="Logger Uno port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--name", default="t870_run", help="Short test condition name")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Capture seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--output-root",
        type=pathlib.Path,
        default=DEFAULT_RUN_ROOT,
        help="Directory that receives timestamped run folders",
    )
    parser.add_argument("--plot", action="store_true", help="Also request diagnosis_plot.png")
    parser.add_argument(
        "--capture-only",
        action="store_true",
        help="Save CSV without running the diagnosis",
    )
    return parser.parse_args()


def write_metadata(path: pathlib.Path, metadata: dict[str, object]) -> None:
    path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    if args.duration < 0:
        print("--duration must be 0 or greater", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print(
            "pyserial is required. Install it with: "
            "python3 -m pip install -r logger/requirements.txt",
            file=sys.stderr,
        )
        return 3

    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_root.resolve() / f"{stamp}_{safe_name(args.name)}"
    report_dir = run_dir / "diagnosis"
    csv_path = run_dir / "raw_can.csv"
    metadata_path = run_dir / "run_metadata.json"
    run_dir.mkdir(parents=True, exist_ok=False)

    started_at = dt.datetime.now().astimezone()
    started_monotonic = time.monotonic()
    header_written = False
    row_count = 0
    complete_row_count = 0
    last_sequence: int | None = None
    stop_reason = "capture_error"
    capture_error = ""
    discarded_count = 0

    print(f"run directory : {run_dir}")
    print(f"logger port   : {args.port} @ {args.baud}")
    if args.duration > 0:
        print(f"capture       : {args.duration:g} seconds")
    else:
        print("capture       : until Ctrl+C")
    print("waiting for logger CSV header...")

    try:
        # Configure DTR/RTS before opening the port. The logger may already be
        # receiving CAN correctly, and a normal pyserial open can pulse DTR and
        # reset an Uno. A capture program must observe it without changing its
        # running state.
        connection = serial.Serial()
        connection.port = args.port
        connection.baudrate = args.baud
        connection.timeout = 1
        connection.dtr = False
        connection.rts = False
        connection.open()
        with connection, csv_path.open("w", encoding="utf-8", newline="") as output:
            try:
                while True:
                    if args.duration > 0 and time.monotonic() - started_monotonic >= args.duration:
                        stop_reason = "duration_elapsed"
                        break

                    raw = connection.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        print(f"logger: {line}", file=sys.stderr)
                        continue
                    if line == LOGGER_HEADER:
                        if not header_written:
                            output.write("host_time_iso," + line + "\n")
                            output.flush()
                            header_written = True
                            print("CSV header received; recording rows.")
                        continue
                    fields = line.split(",")
                    valid_data_row = len(fields) == LOGGER_COLUMN_COUNT
                    if valid_data_row:
                        try:
                            int(fields[0])
                            int(fields[1])
                            int(fields[2])
                        except ValueError:
                            valid_data_row = False
                    if not header_written and valid_data_row:
                        output.write("host_time_iso," + LOGGER_HEADER + "\n")
                        output.flush()
                        header_written = True
                        print("valid CSV row received; recording rows with the known logger header.")
                    if not header_written:
                        discarded_count += 1
                        if discarded_count <= 3:
                            preview = line if len(line) <= 160 else line[:157] + "..."
                            print(f"discarded before header: {preview}", file=sys.stderr)
                        elif discarded_count == 4:
                            print(
                                "additional invalid startup lines are being suppressed.",
                                file=sys.stderr,
                            )
                        continue
                    if not valid_data_row:
                        continue

                    host_time = dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                    output.write(host_time + "," + line + "\n")
                    output.flush()
                    row_count += 1

                    if len(fields) > 2:
                        if fields[1] == "1":
                            complete_row_count += 1
                        try:
                            last_sequence = int(fields[2])
                        except ValueError:
                            pass

                    if row_count == 1 or row_count % 100 == 0:
                        print(
                            f"captured rows={row_count}, complete={complete_row_count}, "
                            f"last_seq={last_sequence}"
                        )
            except KeyboardInterrupt:
                stop_reason = "user_ctrl_c"
                print("\ncapture stopped; starting diagnosis.")
    except (OSError, serial.SerialException) as error:
        capture_error = str(error)
        print(f"serial capture failed: {error}", file=sys.stderr)
        if "PermissionError" in repr(error) or "Access is denied" in str(error):
            print(
                "close Arduino Serial Monitor/Plotter and any program using this port, "
                "then run the command again.",
                file=sys.stderr,
            )

    ended_at = dt.datetime.now().astimezone()
    metadata: dict[str, object] = {
        "test_name": args.name,
        "logger_port": args.port,
        "baud": args.baud,
        "started_at": started_at.isoformat(timespec="milliseconds"),
        "ended_at": ended_at.isoformat(timespec="milliseconds"),
        "duration_s": round(time.monotonic() - started_monotonic, 3),
        "stop_reason": stop_reason,
        "csv_file": str(csv_path),
        "header_received": header_written,
        "row_count": row_count,
        "complete_row_count": complete_row_count,
        "last_sequence": last_sequence,
        "capture_error": capture_error,
        "diagnosis_exit_code": None,
    }

    if capture_error or not header_written or row_count == 0:
        write_metadata(metadata_path, metadata)
        print(f"no usable CSV rows were captured; see {metadata_path}", file=sys.stderr)
        return 4

    if args.capture_only:
        stop_reason = "capture_only"
        metadata["stop_reason"] = stop_reason
        write_metadata(metadata_path, metadata)
        print(f"CSV saved      : {csv_path}")
        print(f"metadata saved : {metadata_path}")
        return 0

    command = [
        sys.executable,
        str(ANALYZER),
        "--input",
        str(csv_path),
        "--output",
        str(report_dir),
    ]
    if not args.plot:
        command.append("--no-plot")

    print("running automatic diagnosis...")
    completed = subprocess.run(command, check=False)
    metadata["diagnosis_exit_code"] = completed.returncode
    write_metadata(metadata_path, metadata)

    print(f"CSV saved      : {csv_path}")
    print(f"metadata saved : {metadata_path}")
    if completed.returncode == 0:
        print(f"diagnosis      : {report_dir / 'diagnosis_summary.md'}")
    else:
        print("diagnosis failed; the original CSV is still safe.", file=sys.stderr)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
