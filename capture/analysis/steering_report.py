"""Compare steering telemetry from repeated runs through the same course."""

import argparse
import csv
import math
from pathlib import Path


def p95(values):
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


def report(path, limit, start, end):
    fields = ("tgtL", "tgtR", "actL", "actR", "err", "turn", "state")
    rows = []
    skipped = 0
    with path.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        missing = set((*fields, "t")) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path}: missing columns {', '.join(sorted(missing))}")
        for raw in reader:
            try:
                time = float(raw["t"])
                row = {name: int(raw[name]) for name in fields}
            except (TypeError, ValueError):
                skipped += 1
                continue
            if start <= time <= end and (row["tgtL"] or row["tgtR"]):
                rows.append((time, row))

    print(f"\n{path} | TMAX={limit:g} | moving samples={len(rows)} | malformed={skipped}")
    print("group          n  |err| p95   |turn| p95   near TMAX   wheel delta error p95")
    for name, state in (("all", None), ("normal", 0), ("corner", 1),
                        ("cross", 2), ("island", 4)):
        group = [row for _, row in rows if state is None or row["state"] == state]
        if not group:
            continue
        turn = [abs(row["turn"]) for row in group]
        delta = [abs((row["actR"] - row["actL"]) -
                     (row["tgtR"] - row["tgtL"])) for row in group]
        near_limit = sum(value >= 0.98 * limit for value in turn)
        print(f"{name:8} {len(group):7d} {p95(abs(r['err']) for r in group)/10:10.1f}"
              f" {p95(turn):12d} {near_limit/len(group):11.1%} {p95(delta):23d}")

    for time, row in sorted(rows, key=lambda item: abs(item[1]["err"]),
                            reverse=True)[:3]:
        print(f"Peak error sample t={time:.3f}s state={row['state']} "
              f"err={row['err']/10:+.1f} turn={row['turn']:+d} "
              f"target={row['tgtL']}/{row['tgtR']} actual={row['actL']}/{row['actR']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--tmax", required=True, type=float, nargs="+",
                        help="Effective TMAX in RPM, one value or one per log. Put after logs.")
    parser.add_argument("--start", type=float, default=0, help="CSV time in seconds")
    parser.add_argument("--end", type=float, default=math.inf, help="CSV time in seconds")
    args = parser.parse_args()
    if len(args.tmax) not in (1, len(args.logs)) or any(
        not math.isfinite(limit) or limit <= 0 for limit in args.tmax
    ):
        parser.error("provide one positive finite TMAX or one per log")
    if not args.start <= args.end:
        parser.error("--start must be at or before --end")
    limits = args.tmax * len(args.logs) if len(args.tmax) == 1 else args.tmax
    for path, limit in zip(args.logs, limits):
        try:
            report(path, limit, args.start, args.end)
        except (OSError, ValueError) as error:
            parser.error(str(error))


if __name__ == "__main__":
    main()
