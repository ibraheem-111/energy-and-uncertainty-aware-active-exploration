import argparse
import csv
import glob
import os


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare cave exploration metric CSV logs."
    )
    parser.add_argument(
        "csv_files",
        nargs="*",
        help="CSV files to compare. Defaults to all CSVs in --log-dir.",
    )
    parser.add_argument(
        "--log-dir",
        default=os.path.expanduser("~/ros2_ws/exploration_logs"),
        help="Directory containing exploration CSV logs.",
    )
    return parser.parse_args()


def read_final_row(path):
    final_row = None
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            final_row = row
    return final_row


def as_float(row, key):
    try:
        return float(row.get(key, "") or 0.0)
    except ValueError:
        return 0.0


def main():
    args = parse_args()
    paths = args.csv_files
    if not paths:
        paths = sorted(glob.glob(os.path.join(args.log_dir, "*.csv")))

    if not paths:
        print(f"No CSV logs found in {args.log_dir}")
        return

    rows = []
    for path in paths:
        final_row = read_final_row(path)
        if final_row is None:
            continue
        known_area = as_float(final_row, "known_area_m2")
        travel = as_float(final_row, "travel_distance_m")
        rows.append({
            "file": os.path.basename(path),
            "label": final_row.get("experiment_label", ""),
            "planner": final_row.get("planner", ""),
            "elapsed": as_float(final_row, "elapsed_s"),
            "timed_out": as_float(final_row, "experiment_timed_out"),
            "known": known_area,
            "free": as_float(final_row, "free_area_m2"),
            "frontiers": as_float(final_row, "frontier_cells"),
            "features": as_float(final_row, "features_detected"),
            "max_features": as_float(final_row, "max_features_detected"),
            "mean_features": as_float(final_row, "mean_features_detected"),
            "travel": travel,
            "efficiency": known_area / travel if travel > 0.0 else 0.0,
        })

    print(
        "file,label,planner,elapsed_s,timed_out,known_area_m2,free_area_m2,"
        "frontier_cells,features_detected,max_features_detected,"
        "mean_features_detected,travel_distance_m,known_area_per_meter"
    )
    for row in rows:
        print(
            f"{row['file']},{row['label']},{row['planner']},"
            f"{row['elapsed']:.1f},{int(row['timed_out'])},"
            f"{row['known']:.2f},{row['free']:.2f},"
            f"{row['frontiers']:.0f},{row['features']:.0f},"
            f"{row['max_features']:.0f},{row['mean_features']:.1f},"
            f"{row['travel']:.2f},"
            f"{row['efficiency']:.2f}"
        )


if __name__ == "__main__":
    main()
