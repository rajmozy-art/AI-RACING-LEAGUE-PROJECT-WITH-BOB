"""
create_results_csvs.py
----------------------
Creates results/lap_data.csv and results/track_data.csv beside this script.
- Uses paths relative to this file, not the CWD.
- Uses Python standard-library csv module only.
- If a file already exists and has data rows, preserves them.
- If a file already exists with an incompatible header, raises an error
  instead of silently replacing it.
- If a file is missing, creates it with the header only (no data rows).
"""

import csv
import os
import sys

# Resolve paths relative to this script, regardless of where Python is run from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")

LAP_DATA_PATH   = os.path.join(RESULTS_DIR, "lap_data.csv")
TRACK_DATA_PATH = os.path.join(RESULTS_DIR, "track_data.csv")

LAP_DATA_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car",
    "lap_number", "lap_time_seconds", "damage_total", "damage_delta",
    "status", "change_description",
]

TRACK_DATA_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car", "lap_number",
    "curLapTime", "lastLapTime", "distFromStart", "distRaced",
    "speedX", "speedY", "angle", "trackPos", "damage",
    "track_0",  "track_1",  "track_2",  "track_3",  "track_4",
    "track_5",  "track_6",  "track_7",  "track_8",  "track_9",
    "track_10", "track_11", "track_12", "track_13", "track_14",
    "track_15", "track_16", "track_17", "track_18",
    "steer", "accel", "brake", "gear",
]


def ensure_csv(path, expected_header):
    """
    Ensure a CSV file exists with the expected header.

    - Missing file   → create with header only.
    - Empty file     → write header only.
    - Header matches → leave untouched (preserves any data rows).
    - Header differs → raise ValueError; do NOT overwrite.
    """
    filename = os.path.basename(path)

    if not os.path.exists(path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        print(f"  CREATED  {path}")
        return

    # File exists — read the first line to check the header.
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            existing_header = next(reader)
        except StopIteration:
            existing_header = []  # Completely empty file.

    if existing_header == expected_header:
        print(f"  OK       {path}  (header matches; file untouched)")
        return

    if existing_header == []:
        # File is empty — safe to write header.
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        print(f"  REPAIRED {path}  (was empty; header written)")
        return

    # Header mismatch — refuse to overwrite.
    raise ValueError(
        f"\nINCOMPATIBLE HEADER in {filename}.\n"
        f"  Expected : {expected_header}\n"
        f"  Found    : {existing_header}\n"
        f"  Action   : rename or delete the existing file manually, then re-run."
    )


if __name__ == "__main__":
    os.makedirs(RESULTS_DIR, exist_ok=True)
    print(f"Results directory: {RESULTS_DIR}\n")
    try:
        ensure_csv(LAP_DATA_PATH,   LAP_DATA_HEADER)
        ensure_csv(TRACK_DATA_PATH, TRACK_DATA_HEADER)
        print("\nAll CSV files are ready.")
    except ValueError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
