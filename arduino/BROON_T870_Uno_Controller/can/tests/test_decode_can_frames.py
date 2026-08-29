from __future__ import annotations

import csv
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest


CAN_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAN_DIR))

from decode_can_frames import (  # noqa: E402
    compare_snapshots,
    decode_frames,
    load_complete_telemetry,
)

try:
    import cantools
except ImportError:  # pragma: no cover - dependency error is reported by the CLI
    cantools = None


@unittest.skipIf(cantools is None, "cantools is not installed")
class DbcDecodeTest(unittest.TestCase):
    def test_dbc_decode_matches_logger_telemetry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            frames_path = directory / "can_frames.csv"
            decoded_path = directory / "dbc_decoded_frames.csv"
            telemetry_path = directory / "raw_can.csv"

            sequence = 7
            frames = [
                (0x100, bytes([sequence, 2, 0, 0, 25]) + struct.pack("<H", 123) + bytes([1])),
                (0x101, bytes([sequence]) + struct.pack("<hhh", 30, 29, 28) + bytes([0])),
                (0x102, bytes([sequence]) + struct.pack("<HHh", 600, 590, 100) + bytes([0])),
                (0x103, bytes([sequence]) + struct.pack("<hi", 12, 5) + bytes([1])),
                (0x104, bytes([sequence]) + struct.pack("<HHH", 1450, 1460, 1500) + bytes([7])),
                (0x105, bytes([sequence]) + struct.pack("<HHBBB", 32000, 0, 0, 0, 0)),
            ]
            with frames_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["host_time_iso", "logger_ms", "can_id", "dlc", "data_hex"])
                for index, (frame_id, payload) in enumerate(frames):
                    writer.writerow(
                        [
                            f"2026-08-29T21:00:00.{index:03d}+09:00",
                            1000 + index * 20,
                            f"0x{frame_id:03X}",
                            len(payload),
                            payload.hex().upper(),
                        ]
                    )

            telemetry_header = [
                "host_time_iso", "logger_ms", "complete", "seq", "protocol",
                "state", "fault", "mode", "status_flags", "uptime_s",
                "drive_req_pwm", "front_pwm", "rear_pwm", "steer_target_adc",
                "steer_actual_adc", "steer_pwm", "speed_kph", "encoder_delta",
                "encoder_corrected", "rc_steer_us", "rc_throttle_us", "rc_aux_us",
                "rc_flags", "rc_read_us", "tx_dropped", "can_eflg", "can_tec", "can_rec",
            ]
            telemetry_values = [
                "2026-08-29T21:00:00.120+09:00", 1120, 1, sequence, 1,
                2, 0, 0, 25, 123, 30, 29, 28, 600, 590, 100, 0.12, 5,
                1, 1450, 1460, 1500, 7, 32000, 0, 0, 0, 0,
            ]
            with telemetry_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(telemetry_header)
                writer.writerow(telemetry_values)

            database = cantools.database.load_file(str(CAN_DIR / "T870_CAN.dbc"), strict=True)
            counts, unknown, errors, snapshots = decode_frames(
                database, frames_path, decoded_path
            )
            telemetry_rows = load_complete_telemetry(telemetry_path)
            compared, fields, unmatched, mismatches = compare_snapshots(
                snapshots, telemetry_rows
            )

            self.assertEqual(sum(counts.values()), 6)
            self.assertEqual(unknown, 0)
            self.assertEqual(errors, [])
            self.assertEqual(compared, 1)
            self.assertEqual(fields, 24)
            self.assertEqual(unmatched, 0)
            self.assertEqual(mismatches, [])

            cli_output = directory / "cli_decoded.csv"
            cli_report = directory / "cli_report.md"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(CAN_DIR / "decode_can_frames.py"),
                    "--input",
                    str(frames_path),
                    "--dbc",
                    str(CAN_DIR / "T870_CAN.dbc"),
                    "--output",
                    str(cli_output),
                    "--telemetry",
                    str(telemetry_path),
                    "--report",
                    str(cli_report),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertTrue(cli_output.exists())
            self.assertIn("판정: **PASS**", cli_report.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
