#!/usr/bin/env python3
"""Merge hardware timing, alarm and operator observations into a stage summary."""

import argparse
import json
from pathlib import Path
import re


TIMING = re.compile(
    r"\[(?P<stamp>\d+\.\d+)\].*Control timing \(1000 cycles\): .*"
    r"max ServoJ=(?P<max>[\d.]+) ms, ServoJ p95=(?P<p95>[\d.]+) ms, "
    r"p99=(?P<p99>[\d.]+) ms, .*ServoJ >10ms=(?P<over10>\d+), "
    r">20ms=(?P<over20>\d+), max consecutive >10ms=(?P<consecutive>\d+)"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True)
    parser.add_argument("--hardware-log", required=True)
    parser.add_argument("--alarm-main", type=int, required=True)
    parser.add_argument("--alarm-sub", type=int, required=True)
    parser.add_argument(
        "--physical-observation",
        choices=("normal", "abnormal", "pending"),
        default="pending",
    )
    parser.add_argument("--start-epoch", type=float)
    parser.add_argument("--end-epoch", type=float)
    return parser.parse_args()


def main():
    args = parse_args()
    summary_path = Path(args.summary)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    start = args.start_epoch or summary.get("stage_started_epoch") or 0.0
    end = args.end_epoch or summary.get("stage_ended_epoch") or float("inf")
    text = Path(args.hardware_log).read_text(encoding="utf-8", errors="replace")
    blocks = []
    for match in TIMING.finditer(text):
        values = match.groupdict()
        stamp = float(values["stamp"])
        if start <= stamp <= end:
            blocks.append({
                "stamp": stamp,
                "max_ms": float(values["max"]),
                "p95_ms": float(values["p95"]),
                "p99_ms": float(values["p99"]),
                "over_10ms": int(values["over10"]),
                "over_20ms": int(values["over20"]),
                "max_consecutive_over_10ms": int(values["consecutive"]),
            })
    selected_text = "\n".join(
        line for line in text.splitlines()
        if not (stamp_match := re.search(r"\[(\d+\.\d+)\]", line))
        or start <= float(stamp_match.group(1)) <= end
    )
    timing = {
        "sample_blocks": len(blocks),
        "sample_count": len(blocks) * 1000,
        "max_ms": max((block["max_ms"] for block in blocks), default=None),
        "worst_block_p95_ms": max((block["p95_ms"] for block in blocks), default=None),
        "worst_block_p99_ms": max((block["p99_ms"] for block in blocks), default=None),
        "over_10ms": sum(block["over_10ms"] for block in blocks),
        "over_20ms": sum(block["over_20ms"] for block in blocks),
        "max_consecutive_over_10ms": max(
            (block["max_consecutive_over_10ms"] for block in blocks), default=0
        ),
        "note": "P95/P99 are the worst 1000-call block percentiles.",
    }
    summary["servo_timing"] = timing
    summary["robot_alarm"] = {"main": args.alarm_main, "sub": args.alarm_sub}
    summary["path_tolerance_violated"] = "PATH_TOLERANCE_VIOLATED" in selected_text
    summary["servo_code_14"] = bool(re.search(
        r"ServoJ .*?(?:SDK )?code 14|ServoJ rejected.*code 14", selected_text
    ))
    summary["physical_observation"] = args.physical_observation
    summary["final_pass"] = bool(
        summary.get("pass_from_action_and_tracking")
        and timing["max_ms"] is not None
        and timing["max_ms"] <= 20.0
        and timing["over_20ms"] == 0
        and timing["max_consecutive_over_10ms"] < 2
        and not summary["path_tolerance_violated"]
        and not summary["servo_code_14"]
        and args.alarm_main == 0
        and args.alarm_sub == 0
        and args.physical_observation == "normal"
    )
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
