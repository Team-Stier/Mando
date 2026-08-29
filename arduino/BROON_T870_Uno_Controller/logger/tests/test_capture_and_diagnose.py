from __future__ import annotations

import pathlib
import sys
import unittest


LOGGER_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOGGER_DIR))

from capture_and_diagnose import parse_raw_can_line  # noqa: E402


class RawCanLineParserTest(unittest.TestCase):
    def test_accepts_and_normalizes_standard_frame(self) -> None:
        self.assertEqual(
            parse_raw_can_line("@CAN,2254036,0x102,8,3a01e803d2000000"),
            ("2254036", "0x102", "8", "3A01E803D2000000"),
        )

    def test_rejects_bad_dlc_and_extended_id(self) -> None:
        self.assertIsNone(parse_raw_can_line("@CAN,1,0x102,8,0102"))
        self.assertIsNone(parse_raw_can_line("@CAN,1,0x800,0,"))

    def test_rejects_non_can_serial_line(self) -> None:
        self.assertIsNone(parse_raw_can_line("1,1,2,1,0,0"))


if __name__ == "__main__":
    unittest.main()
