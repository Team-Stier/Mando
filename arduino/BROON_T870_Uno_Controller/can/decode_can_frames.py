#!/usr/bin/env python3
"""Decode a T870 raw CAN capture with its DBC and cross-check telemetry CSV."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import sys
from collections import Counter, defaultdict
from typing import Any


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
DEFAULT_DBC = SCRIPT_DIR / "T870_CAN.dbc"
EXPECTED_FRAME_IDS = tuple(range(0x100, 0x106))
MAX_CAPTURE_BOUNDARY_UNMATCHED_ROWS = 2
RAW_FRAME_COLUMNS = ("host_time_iso", "logger_ms", "can_id", "dlc", "data_hex")

# The DBC signal is the source of truth on the left. The decoded logger column
# on the right is independently produced by T870CanCsvLogger. Comparing them
# proves that both implementations interpret the same CAN bytes identically.
SIGNAL_TO_TELEMETRY_COLUMN = {
    "ProtocolVersion": "protocol",
    "State": "state",
    "Fault": "fault",
    "Mode": "mode",
    "StatusFlags": "status_flags",
    "UptimeS": "uptime_s",
    "RequestedPwm": "drive_req_pwm",
    "FrontPwm": "front_pwm",
    "RearPwm": "rear_pwm",
    "TargetAdc": "steer_target_adc",
    "ActualAdc": "steer_actual_adc",
    "SteeringPwm": "steer_pwm",
    "SpeedKph": "speed_kph",
    "EncoderDelta": "encoder_delta",
    "EncoderCalibrated": "encoder_corrected",
    "SteerPulseUs": "rc_steer_us",
    "ThrottlePulseUs": "rc_throttle_us",
    "AuxPulseUs": "rc_aux_us",
    "RcFlags": "rc_flags",
    "RcReadUs": "rc_read_us",
    "DroppedFrames": "tx_dropped",
    "CanErrorFlags": "can_eflg",
    "CanTxErrorCount": "can_tec",
    "CanRxErrorCount": "can_rec",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Decode can_frames.csv with T870_CAN.dbc and optionally compare "
            "the result with the logger's decoded raw_can.csv."
        )
    )
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--dbc", type=pathlib.Path, default=DEFAULT_DBC)
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        help="Decoded frame CSV; default: INPUT directory/dbc_decoded_frames.csv",
    )
    parser.add_argument(
        "--telemetry",
        type=pathlib.Path,
        help="Decoded logger CSV to cross-check; default: sibling raw_can.csv if present",
    )
    parser.add_argument(
        "--report",
        type=pathlib.Path,
        help="Markdown report; default: INPUT directory/dbc_verification_summary.md",
    )
    return parser.parse_args()


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_frame_row(row: dict[str, str], row_number: int) -> tuple[int, int, bytes]:
    try:
        frame_id = int(row["can_id"], 0)
        dlc = int(row["dlc"], 10)
        payload = bytes.fromhex(row["data_hex"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"row {row_number}: invalid CAN frame fields: {error}") from error

    if not 0 <= frame_id <= 0x7FF:
        raise ValueError(f"row {row_number}: standard CAN ID out of range: {frame_id}")
    if not 0 <= dlc <= 8:
        raise ValueError(f"row {row_number}: DLC out of range: {dlc}")
    if len(payload) != dlc:
        raise ValueError(
            f"row {row_number}: DLC={dlc} but data contains {len(payload)} bytes"
        )
    return frame_id, dlc, payload


def occurrence_keys(rows: list[dict[str, Any]]) -> dict[tuple[int, int], dict[str, Any]]:
    counts: defaultdict[int, int] = defaultdict(int)
    keyed: dict[tuple[int, int], dict[str, Any]] = {}
    for row in rows:
        sequence = int(row["sequence"])
        key = (sequence, counts[sequence])
        counts[sequence] += 1
        keyed[key] = row
    return keyed


def numeric_equal(signal: str, dbc_value: Any, telemetry_value: str) -> bool:
    try:
        left = float(dbc_value)
        right = float(telemetry_value)
    except (TypeError, ValueError):
        return str(dbc_value) == str(telemetry_value)
    tolerance = 0.0051 if signal == "SpeedKph" else 0.0
    return abs(left - right) <= tolerance


def decode_frames(
    database: Any,
    input_path: pathlib.Path,
    output_path: pathlib.Path,
) -> tuple[
    Counter[int],
    int,
    list[str],
    list[dict[str, Any]],
]:
    frame_counts: Counter[int] = Counter()
    unknown_frame_count = 0
    decode_errors: list[str] = []
    snapshots: list[dict[str, Any]] = []
    current_sequence: int | None = None
    current_signals: dict[str, Any] = {}
    current_seen_ids: set[int] = set()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with input_path.open("r", encoding="utf-8-sig", newline="") as source, output_path.open(
        "w", encoding="utf-8", newline=""
    ) as destination:
        reader = csv.DictReader(source)
        missing_columns = set(RAW_FRAME_COLUMNS) - set(reader.fieldnames or ())
        if missing_columns:
            raise ValueError(
                "raw CAN CSV is missing columns: " + ", ".join(sorted(missing_columns))
            )

        writer = csv.DictWriter(
            destination,
            fieldnames=[
                *RAW_FRAME_COLUMNS,
                "message_name",
                "sequence",
                "decoded_signals_json",
            ],
        )
        writer.writeheader()

        for row_number, row in enumerate(reader, start=2):
            try:
                frame_id, dlc, payload = parse_frame_row(row, row_number)
            except ValueError as error:
                decode_errors.append(str(error))
                continue

            frame_counts[frame_id] += 1
            try:
                message = database.get_message_by_frame_id(frame_id)
            except KeyError:
                unknown_frame_count += 1
                writer.writerow(
                    {
                        **{column: row[column] for column in RAW_FRAME_COLUMNS},
                        "message_name": "UNKNOWN",
                        "sequence": "",
                        "decoded_signals_json": "{}",
                    }
                )
                continue

            try:
                decoded = message.decode(payload, decode_choices=False, scaling=True)
                sequence = int(decoded["Sequence"])
            except (KeyError, TypeError, ValueError) as error:
                decode_errors.append(
                    f"row {row_number}: 0x{frame_id:03X} decode failed: {error}"
                )
                continue

            writer.writerow(
                {
                    **{column: row[column] for column in RAW_FRAME_COLUMNS},
                    "message_name": message.name,
                    "sequence": sequence,
                    "decoded_signals_json": json.dumps(
                        decoded, ensure_ascii=False, sort_keys=True, separators=(",", ":")
                    ),
                }
            )

            if current_sequence != sequence:
                current_sequence = sequence
                current_signals = {}
                current_seen_ids = set()
            current_signals.update(decoded)
            current_seen_ids.add(frame_id)

            if frame_id == 0x105:
                snapshots.append(
                    {
                        "sequence": sequence,
                        "signals": dict(current_signals),
                        "seen_ids": set(current_seen_ids),
                        "complete": set(EXPECTED_FRAME_IDS).issubset(current_seen_ids),
                    }
                )

    return frame_counts, unknown_frame_count, decode_errors, snapshots


def load_complete_telemetry(path: pathlib.Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"seq", "complete", *SIGNAL_TO_TELEMETRY_COLUMN.values()}
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(
                "telemetry CSV is missing columns: " + ", ".join(sorted(missing))
            )
        for row in reader:
            if row["complete"] != "1":
                continue
            row["sequence"] = int(row["seq"])
            rows.append(row)
    return rows


def compare_snapshots(
    snapshots: list[dict[str, Any]], telemetry_rows: list[dict[str, Any]]
) -> tuple[int, int, int, list[str]]:
    complete_snapshots = [snapshot for snapshot in snapshots if snapshot["complete"]]
    dbc_by_key = occurrence_keys(complete_snapshots)
    telemetry_by_key = occurrence_keys(telemetry_rows)
    shared_keys = sorted(set(dbc_by_key) & set(telemetry_by_key))
    mismatches: list[str] = []
    field_comparison_count = 0

    for key in shared_keys:
        snapshot = dbc_by_key[key]
        telemetry = telemetry_by_key[key]
        signals = snapshot["signals"]
        for signal, column in SIGNAL_TO_TELEMETRY_COLUMN.items():
            field_comparison_count += 1
            if signal not in signals or not numeric_equal(signal, signals[signal], telemetry[column]):
                mismatches.append(
                    f"seq={key[0]} occurrence={key[1]} {signal}/{column}: "
                    f"DBC={signals.get(signal)!r}, logger={telemetry[column]!r}"
                )

    unmatched_count = len(set(dbc_by_key) ^ set(telemetry_by_key))
    return len(shared_keys), field_comparison_count, unmatched_count, mismatches


def write_report(
    report_path: pathlib.Path,
    input_path: pathlib.Path,
    dbc_path: pathlib.Path,
    output_path: pathlib.Path,
    telemetry_path: pathlib.Path | None,
    frame_counts: Counter[int],
    unknown_frame_count: int,
    decode_errors: list[str],
    snapshots: list[dict[str, Any]],
    compared_rows: int,
    field_comparisons: int,
    unmatched_rows: int,
    mismatches: list[str],
) -> None:
    expected_missing = [frame_id for frame_id in EXPECTED_FRAME_IDS if frame_counts[frame_id] == 0]
    complete_snapshots = sum(1 for snapshot in snapshots if snapshot["complete"])
    comparison_required = telemetry_path is not None
    passed = (
        not decode_errors
        and not expected_missing
        and (
            not comparison_required
            or (
                compared_rows > 0
                and not mismatches
                and unmatched_rows <= MAX_CAPTURE_BOUNDARY_UNMATCHED_ROWS
            )
        )
    )

    lines = [
        "# BROON T870 DBC 재해석 검증 보고서",
        "",
        f"- 판정: **{'PASS' if passed else 'CHECK'}**",
        f"- 원본 프레임: `{input_path}`",
        f"- DBC: `{dbc_path}`",
        f"- DBC 해석 결과: `{output_path}`",
        f"- 원본 프레임 SHA-256: `{sha256_file(input_path)}`",
        f"- DBC SHA-256: `{sha256_file(dbc_path)}`",
        "",
        "## CAN ID별 프레임 수",
        "",
        "| CAN ID | DBC 메시지 | 프레임 수 |",
        "|---:|---|---:|",
    ]
    for frame_id in EXPECTED_FRAME_IDS:
        try:
            message_name = {
                0x100: "T870_STATUS",
                0x101: "T870_DRIVE",
                0x102: "T870_STEERING",
                0x103: "T870_MOTION",
                0x104: "T870_RC",
                0x105: "T870_TIMING",
            }[frame_id]
        except KeyError:
            message_name = "UNKNOWN"
        lines.append(f"| `0x{frame_id:03X}` | {message_name} | {frame_counts[frame_id]} |")

    lines.extend(
        [
            "",
            "## 해석 결과",
            "",
            f"- 전체 원본 프레임: {sum(frame_counts.values())}",
            f"- DBC 미정의 프레임: {unknown_frame_count}",
            f"- DBC 해석 오류: {len(decode_errors)}",
            f"- 조립된 sequence: {len(snapshots)}",
            f"- 6개 ID가 모두 있는 완전 sequence: {complete_snapshots}",
        ]
    )

    if telemetry_path is not None:
        lines.extend(
            [
                "",
                "## Arduino 로거 해석값 교차검증",
                "",
                f"- 비교 대상: `{telemetry_path}`",
                f"- 비교한 완전 행: {compared_rows}",
                f"- 비교한 신호 값: {field_comparisons}",
                f"- 대응되지 않은 행: {unmatched_rows}",
                "- 캡처 시작·종료 경계 허용 행: "
                f"{MAX_CAPTURE_BOUNDARY_UNMATCHED_ROWS}",
                f"- 값 불일치: {len(mismatches)}",
            ]
        )

    if expected_missing:
        lines.extend(
            [
                "",
                "## 누락된 필수 CAN ID",
                "",
                *[f"- `0x{frame_id:03X}`" for frame_id in expected_missing],
            ]
        )
    if decode_errors:
        lines.extend(["", "## 해석 오류", "", *[f"- {item}" for item in decode_errors[:20]]])
    if mismatches:
        lines.extend(["", "## 값 불일치", "", *[f"- {item}" for item in mismatches[:20]]])

    lines.extend(
        [
            "",
            "## 판정 의미",
            "",
            "PASS는 원본 CAN 바이트를 DBC로 독립 해석한 값과 로거 Uno가 해석한 값이 "
            "일치한다는 의미이다. 차량이나 센서 자체의 무고장을 보증하는 판정은 아니다.",
            "",
        ]
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_path = args.input.resolve()
    dbc_path = args.dbc.resolve()
    output_path = (args.output or input_path.with_name("dbc_decoded_frames.csv")).resolve()
    report_path = (args.report or input_path.with_name("dbc_verification_summary.md")).resolve()
    telemetry_path = args.telemetry
    if telemetry_path is None:
        candidate = input_path.with_name("raw_can.csv")
        telemetry_path = candidate if candidate.exists() else None
    elif not telemetry_path.exists():
        print(f"telemetry CSV not found: {telemetry_path}", file=sys.stderr)
        return 2
    if telemetry_path is not None:
        telemetry_path = telemetry_path.resolve()

    try:
        import cantools
    except ImportError:
        print(
            "cantools is required. Install logger dependencies with: "
            "python3 -m pip install -r logger/requirements.txt",
            file=sys.stderr,
        )
        return 3

    try:
        database = cantools.database.load_file(str(dbc_path), strict=True)
        frame_counts, unknown_count, decode_errors, snapshots = decode_frames(
            database, input_path, output_path
        )
        compared_rows = 0
        field_comparisons = 0
        unmatched_rows = 0
        mismatches: list[str] = []
        if telemetry_path is not None:
            telemetry_rows = load_complete_telemetry(telemetry_path)
            compared_rows, field_comparisons, unmatched_rows, mismatches = compare_snapshots(
                snapshots, telemetry_rows
            )
        write_report(
            report_path,
            input_path,
            dbc_path,
            output_path,
            telemetry_path,
            frame_counts,
            unknown_count,
            decode_errors,
            snapshots,
            compared_rows,
            field_comparisons,
            unmatched_rows,
            mismatches,
        )
    except (OSError, ValueError) as error:
        print(f"DBC verification failed: {error}", file=sys.stderr)
        return 4

    expected_missing = [frame_id for frame_id in EXPECTED_FRAME_IDS if frame_counts[frame_id] == 0]
    failed = bool(
        decode_errors
        or expected_missing
        or mismatches
        or unmatched_rows > MAX_CAPTURE_BOUNDARY_UNMATCHED_ROWS
    )
    if telemetry_path is not None and compared_rows == 0:
        failed = True

    print(f"decoded frames : {output_path}")
    print(f"verification   : {report_path}")
    print(
        f"frames={sum(frame_counts.values())}, complete_sequences="
        f"{sum(1 for snapshot in snapshots if snapshot['complete'])}, "
        f"compared_rows={compared_rows}, mismatches={len(mismatches)}"
    )
    return 5 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
