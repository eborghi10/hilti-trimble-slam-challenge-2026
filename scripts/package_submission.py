#!/usr/bin/env python3
"""Package trajectory .txt files into a submission zip for the Hilti-Trimble Challenge 2026."""

import argparse
import os
import re
import zipfile
import sys

EXPECTED_RUNS = [
    "floor_1_2025-05-05_run_1",
    "floor_1_2025-07-07_run_1",
    "floor_1_2025-12-02_run_1",
    "floor_2_2025-05-05_run_1",
    "floor_2_2025-10-28_run_1",
    "floor_2_2025-10-28_run_2",
    "floor_2_2025-12-02_run_1",
    "floor_2_2025-12-03_run_1",
    "floor_3_2025-05-19_run_1",
    "floor_3_2025-12-02_run_1",
    "floor_4_2025-05-19_run_1",
    "floor_4_2025-12-02_run_1",
    "floor_5_2025-12-02_run_1",
    "floor_6_2025-06-18_run_1",
    "floor_6_2025-07-07_run_1",
    "floor_6_2025-12-02_run_1",
    "floor_6_2025-12-02_run_2",
    "floor_7_2025-12-02_run_1",
    "floor_7_2025-12-02_run_2",
    "floor_7_2025-12-03_run_1",
    "floor_EG_2025-10-16_run_1",
    "floor_EG_2025-12-02_run_1",
    "floor_EG_2025-12-02_run_2",
    "floor_UG1_2025-05-19_run_1",
    "floor_UG1_2025-06-18_run_1",
    "floor_UG1_2025-10-16_run_1",
    "floor_UG1_2025-12-02_run_1",
    "floor_UG1_2025-12-02_run_2",
    "floor_UG1_2025-12-03_run_1",
    "floor_UG2_2025-12-02_run_1",
]

FILENAME_RE = re.compile(r"^floor_[A-Za-z0-9]+_\d{4}-\d{2}-\d{2}_run_\d+\.txt$")


def validate_file(path):
    """Basic validation of a TUM-format trajectory file."""
    lines = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                return False, f"Line {lines+1}: expected 8 values, got {len(parts)}"
            try:
                [float(x) for x in parts]
            except ValueError:
                return False, f"Line {lines+1}: non-numeric value"
            lines += 1
    if lines == 0:
        return False, "File is empty (no pose lines)"
    return True, f"{lines} poses"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", help="Directory containing floor_X_YYYY-MM-DD_run_Z.txt files")
    parser.add_argument("-o", "--output", default="submission.zip", help="Output zip path")
    parser.add_argument("--task", choices=["slam", "localization", "both"], default="both",
                        help="Which task to package for (default: both = all 30 runs)")
    parser.add_argument("--no-validate", action="store_true", help="Skip file validation")
    args = parser.parse_args()

    if not os.path.isdir(args.input_dir):
        print(f"Error: {args.input_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    # Find matching files
    txt_files = sorted(f for f in os.listdir(args.input_dir) if f.endswith(".txt"))
    valid_files = [f for f in txt_files if FILENAME_RE.match(f)]

    if not valid_files:
        print(f"Error: No valid trajectory files found in {args.input_dir}", file=sys.stderr)
        print(f"Expected format: floor_X_YYYY-MM-DD_run_Z.txt", file=sys.stderr)
        sys.exit(1)

    # Check against expected runs
    found_stems = {os.path.splitext(f)[0] for f in valid_files}
    missing = [r for r in EXPECTED_RUNS if r not in found_stems]
    extra = found_stems - set(EXPECTED_RUNS)

    print(f"Found {len(valid_files)} trajectory files")
    if missing:
        print(f"WARNING: {len(missing)} expected runs missing:")
        for m in missing:
            print(f"  - {m}.txt")
    if extra:
        print(f"NOTE: {len(extra)} extra files (not in expected list, will still be included):")
        for e in sorted(extra):
            print(f"  + {e}.txt")

    # Validate
    if not args.no_validate:
        errors = []
        for f in valid_files:
            ok, msg = validate_file(os.path.join(args.input_dir, f))
            if not ok:
                errors.append((f, msg))
            else:
                print(f"  ✓ {f} ({msg})")
        if errors:
            print(f"\nERROR: {len(errors)} file(s) failed validation:")
            for fname, msg in errors:
                print(f"  ✗ {fname}: {msg}")
            sys.exit(1)

    # Create zip
    with zipfile.ZipFile(args.output, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in valid_files:
            zf.write(os.path.join(args.input_dir, f), f)

    print(f"\n✓ Created {args.output} with {len(valid_files)} files")
    print(f"  Upload to: https://submit.hilti-challenge.com/submission/new")


if __name__ == "__main__":
    main()
