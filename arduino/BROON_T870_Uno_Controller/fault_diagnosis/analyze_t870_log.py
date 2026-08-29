#!/usr/bin/env python3
"""CLI for post-drive BROON T870 CAN log diagnosis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from diagnosis import analyze_csv, load_config
from report_generator import create_plot, write_diagnosis_data, write_events_csv, write_summary


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "config" / "thresholds.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze a T870 CAN logger CSV and create a rule-based fault report."
    )
    parser.add_argument("--input", type=Path, required=True, help="T870 CAN logger CSV")
    parser.add_argument("--output", type=Path, required=True, help="Output report directory")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Threshold JSON")
    parser.add_argument("--no-plot", action="store_true", help="Do not create diagnosis_plot.png")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config)
        result = analyze_csv(args.input, config)
    except (OSError, ValueError) as error:
        print(f"analysis failed: {error}", file=sys.stderr)
        return 2

    args.output.mkdir(parents=True, exist_ok=True)
    write_events_csv(result, args.output / "diagnosis_events.csv")
    write_diagnosis_data(result, args.output / "diagnosis_data.csv")
    plot_created = False
    if not args.no_plot:
        plot_created = create_plot(result, args.output / "diagnosis_plot.png")
    write_summary(
        result,
        args.input,
        args.output / "diagnosis_summary.md",
        plot_created=plot_created,
    )

    print(f"analyzed rows: {len(result.rows)}")
    print(f"diagnostic events: {len(result.events)}")
    print(f"report directory: {args.output}")
    if not args.no_plot and not plot_created:
        print("plot skipped: install matplotlib to create diagnosis_plot.png", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
