from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from diagnosis import REQUIRED_COLUMNS, analyze_csv, load_config  # noqa: E402
from report_generator import write_diagnosis_data, write_events_csv, write_summary  # noqa: E402


CONFIG_PATH = PROJECT_DIR / "config" / "thresholds.json"


def normal_row(index: int) -> dict[str, object]:
    return {
        "host_time_iso": f"2026-08-24T12:00:{index // 8:02d}.{(index % 8) * 120:03d}+09:00",
        "logger_ms": index * 120,
        "complete": 1,
        "seq": index % 256,
        "protocol": 1,
        "state": 2,
        "fault": 0,
        "mode": 0,
        "status_flags": (1 << 0) | (1 << 3) | (1 << 4),
        "uptime_s": index * 120 // 1000,
        "drive_req_pwm": 0,
        "front_pwm": 0,
        "rear_pwm": 0,
        "steer_target_adc": 600,
        "steer_actual_adc": 600,
        "steer_pwm": 0,
        "speed_kph": 0.0,
        "encoder_delta": 0,
        "encoder_corrected": 1,
        "rc_steer_us": 1450,
        "rc_throttle_us": 1450,
        "rc_aux_us": 1700,
        "rc_flags": 0b0111,
        "rc_read_us": 5000,
        "tx_dropped": 0,
        "can_eflg": 0,
        "can_tec": 0,
        "can_rec": 0,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


class DiagnosisTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(CONFIG_PATH)

    def analyze(self, rows: list[dict[str, object]]):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "input.csv"
        write_csv(path, rows)
        return analyze_csv(path, self.config), path, Path(temporary.name)

    def test_normal_log_has_no_warning_or_high_event(self) -> None:
        result, _, _ = self.analyze([normal_row(index) for index in range(20)])
        severe = [event for event in result.events if event.severity in {"HIGH", "MEDIUM"}]
        self.assertEqual(severe, [])

    def test_detects_steering_no_response(self) -> None:
        rows = [normal_row(index) for index in range(24)]
        for row in rows[5:20]:
            row["steer_target_adc"] = 900
            row["steer_actual_adc"] = 600
            row["steer_pwm"] = 160
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_TRACKING_ERROR", codes)
        self.assertIn("STEERING_NO_RESPONSE", codes)

    def test_does_not_infer_steering_response_fault_during_stop(self) -> None:
        rows = [normal_row(index) for index in range(24)]
        for row in rows[4:20]:
            row["status_flags"] = (1 << 1) | (1 << 2) | (1 << 4)
            row["steer_target_adc"] = 900
            row["steer_actual_adc"] = 600
            row["steer_pwm"] = 160
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertNotIn("STEERING_TRACKING_ERROR", codes)
        self.assertNotIn("STEERING_NO_RESPONSE", codes)
        self.assertNotIn("STEERING_REVERSE_RESPONSE", codes)
        self.assertNotIn("STEERING_SENSOR_STUCK", codes)

    def test_detects_steering_reverse_response(self) -> None:
        rows = [normal_row(index) for index in range(28)]
        actual = 600
        for row in rows[5:22]:
            actual -= 5
            row["steer_target_adc"] = 900
            row["steer_actual_adc"] = actual
            row["steer_pwm"] = -160
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_REVERSE_RESPONSE", codes)

    def test_detects_steering_sensor_jump_and_safe_range(self) -> None:
        rows = [normal_row(index) for index in range(12)]
        rows[5]["steer_actual_adc"] = 900
        for row in rows[6:11]:
            row["steer_actual_adc"] = 1021
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_SENSOR_JUMP", codes)
        self.assertIn("STEERING_SAFE_RANGE", codes)

    def test_accepts_fast_commanded_steering_response(self) -> None:
        rows = [normal_row(index) for index in range(20)]
        rows[5]["steer_target_adc"] = 1000
        rows[5]["steer_actual_adc"] = 650
        rows[5]["steer_pwm"] = -210
        rows[6]["steer_target_adc"] = 1000
        rows[6]["steer_actual_adc"] = 950
        rows[6]["steer_pwm"] = -210
        for row in rows[7:]:
            row["steer_target_adc"] = 1000
            row["steer_actual_adc"] = 999
            row["steer_pwm"] = 0
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertNotIn("STEERING_SENSOR_JUMP", codes)
        self.assertNotIn("STEERING_TRACKING_ERROR", codes)
        self.assertNotIn("STEERING_SENSOR_STUCK", codes)

    def test_accepts_arrival_sample_after_steering_pwm_turns_off(self) -> None:
        rows = [normal_row(index) for index in range(14)]
        rows[5]["steer_target_adc"] = 480
        rows[5]["steer_actual_adc"] = 656
        rows[5]["steer_pwm"] = 210
        rows[6]["steer_target_adc"] = 413
        rows[6]["steer_actual_adc"] = 405
        rows[6]["steer_pwm"] = 0
        for row in rows[7:]:
            row["steer_target_adc"] = 413
            row["steer_actual_adc"] = 410
            row["steer_pwm"] = 0
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertNotIn("STEERING_SENSOR_JUMP", codes)

    def test_still_detects_jump_after_previous_pwm_in_wrong_direction(self) -> None:
        rows = [normal_row(index) for index in range(14)]
        rows[5]["steer_target_adc"] = 480
        rows[5]["steer_actual_adc"] = 656
        rows[5]["steer_pwm"] = 210
        rows[6]["steer_target_adc"] = 480
        rows[6]["steer_actual_adc"] = 930
        rows[6]["steer_pwm"] = 0
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_SENSOR_JUMP", codes)

    def test_detects_steering_sensor_stuck_across_command_changes(self) -> None:
        rows = [normal_row(index) for index in range(30)]
        for index, row in enumerate(rows[4:24], start=4):
            row["steer_target_adc"] = 900 if index % 2 == 0 else 300
            row["steer_actual_adc"] = 600
            row["steer_pwm"] = 160 if index % 2 == 0 else -160
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_SENSOR_STUCK", codes)

    def test_detects_steering_hunting_with_actual_oscillation(self) -> None:
        rows = [normal_row(index) for index in range(24)]
        for index, row in enumerate(rows[4:20], start=4):
            row["steer_target_adc"] = 600
            row["steer_actual_adc"] = 580 if index % 2 == 0 else 620
            row["steer_pwm"] = 100 if index % 2 == 0 else -100
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("STEERING_HUNTING", codes)

    def test_accepts_capture_start_and_observed_idle_sample_period(self) -> None:
        rows = [normal_row(index) for index in range(8)]
        for index, row in enumerate(rows):
            row["logger_ms"] = index * 647
            row["seq"] = 14 + index
        rows[0]["complete"] = 0
        rows[0]["protocol"] = 0
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertNotIn("CAN_INCOMPLETE_SAMPLE", codes)
        self.assertNotIn("CAN_SAMPLE_GAP", codes)

    def test_detects_drive_no_feedback_and_overspeed(self) -> None:
        rows = [normal_row(index) for index in range(30)]
        for row in rows[4:22]:
            row["drive_req_pwm"] = 80
            row["front_pwm"] = 80
            row["rear_pwm"] = 80
            row["speed_kph"] = 0.0
            row["encoder_delta"] = 0
        rows[25]["speed_kph"] = 15.8
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("DRIVE_NO_FEEDBACK", codes)
        self.assertIn("DRIVE_OVERSPEED", codes)

    def test_detects_can_and_rc_faults(self) -> None:
        rows = [normal_row(index) for index in range(60)]
        rows[3]["complete"] = 0
        rows[4]["seq"] = 9
        rows[6]["tx_dropped"] = 2
        rows[7]["tx_dropped"] = 2
        rows[50]["can_eflg"] = 32
        for row in rows[5:10]:
            row["rc_flags"] = 0b0011
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertIn("CAN_INCOMPLETE_SAMPLE", codes)
        self.assertIn("CAN_SEQUENCE_GAP", codes)
        self.assertIn("CAN_TX_DROP", codes)
        self.assertIn("CAN_ERROR_FLAG", codes)
        self.assertIn("RC_SIGNAL_INVALID", codes)

    def test_ignores_can_error_counter_recovery_during_startup(self) -> None:
        rows = [normal_row(index) for index in range(60)]
        for index, row in enumerate(rows[:40]):
            row["can_eflg"] = 5 if index < 8 else 0
            row["can_tec"] = max(0, 118 - index * 6)
        result, _, _ = self.analyze(rows)
        codes = {event.code for event in result.events}
        self.assertNotIn("CAN_ERROR_FLAG", codes)
        self.assertNotIn("CAN_TEC_HIGH", codes)

    def test_writes_all_text_outputs(self) -> None:
        rows = [normal_row(index) for index in range(12)]
        rows[5]["fault"] = 3
        rows[5]["state"] = 3
        result, input_path, output_dir = self.analyze(rows)
        events_path = output_dir / "diagnosis_events.csv"
        data_path = output_dir / "diagnosis_data.csv"
        summary_path = output_dir / "diagnosis_summary.md"
        write_events_csv(result, events_path)
        write_diagnosis_data(result, data_path)
        write_summary(result, input_path, summary_path, plot_created=False)
        self.assertTrue(events_path.exists())
        self.assertTrue(data_path.exists())
        self.assertIn("조향모터 스톨", summary_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
