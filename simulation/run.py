"""Run from repository root: python3 -m simulation.run."""

import argparse
import csv
import json
from pathlib import Path
from .inputs import source_summary
from .model import simulate
from .mission import build_mission, mission_rows


def main():
    p = argparse.ArgumentParser(
        description="TUSI recovery physics model (uncalibrated)"
    )
    p.add_argument("--config", type=Path)
    p.add_argument("--initialization", choices=["ork", "measured", "theoretical"])
    p.add_argument("--mode", choices=["target", "manual", "neutral", "heading"])
    p.add_argument("--output", type=Path, default=Path("simulation/results"))
    p.add_argument(
        "--sweep",
        action="store_true",
        help="Compare right-motor angle at the same conditions",
    )
    args = p.parse_args()
    config = json.loads(args.config.read_text()) if args.config else {}
    for key in ("initialization", "mode"):
        if getattr(args, key):
            config[key] = getattr(args, key)
    result = simulate(config)
    result["mission"] = build_mission(result)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )
    (args.output / "sources.json").write_text(
        json.dumps(source_summary(), ensure_ascii=False, indent=2, allow_nan=False)
        + "\n"
    )
    with (args.output / "trajectory.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=result["trajectory"][0].keys())
        writer.writeheader()
        writer.writerows(result["trajectory"])
    with (args.output / "control_log.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=result["control_log"][0].keys())
        writer.writeheader()
        writer.writerows(result["control_log"])
    with (args.output / "mission.csv").open("w", newline="") as f:
        rows = iter(mission_rows(result))
        first = next(rows)
        writer = csv.DictWriter(f, fieldnames=first.keys())
        writer.writeheader()
        writer.writerow(first)
        writer.writerows(rows)
    if args.sweep:
        summaries = []
        for angle in [0, 45, 90, 135, 180, 225, 270]:
            if angle > result["config"]["max_angle_deg"]:
                continue
            r = simulate(
                {
                    **config,
                    "mode": "manual",
                    "left_command_deg": 0,
                    "right_command_deg": angle,
                }
            )
            summaries.append({"right_command_deg": angle, **r["summary"]})
        with (args.output / "angle_sweep.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=summaries[0].keys())
            writer.writeheader()
            writer.writerows(summaries)
    print(json.dumps(result["summary"], ensure_ascii=False, indent=2))
    print("UNVALIDATED assumptions. Results:", args.output.resolve())


if __name__ == "__main__":
    main()
