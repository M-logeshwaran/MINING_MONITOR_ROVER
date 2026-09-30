#!/usr/bin/env python3
# ============================================================
# TEST 1 — SOURCE INTEGRITY TEST
# Verifies that no important source code files in rover_ws/src,
# station_ws/src, scripts, firmware, or docs are 0 bytes.
# ============================================================

import os
import sys

PROTOTYPE_ROOT = os.environ.get("DRILLPULSE_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SOURCE_DIRS = [
    os.path.join(PROTOTYPE_ROOT, "rover_ws", "src"),
    os.path.join(PROTOTYPE_ROOT, "station_ws", "src"),
    os.path.join(PROTOTYPE_ROOT, "scripts"),
    os.path.join(PROTOTYPE_ROOT, "firmware"),
    os.path.join(PROTOTYPE_ROOT, "docs")
]

def check_source_integrity():
    print("\n--- TEST 1: SOURCE INTEGRITY CHECK ---")
    zero_byte_files = []
    checked_count = 0

    for sdir in SOURCE_DIRS:
        if not os.path.exists(sdir):
            continue
        for root, dirs, files in os.walk(sdir):
            for fname in files:
                fpath = os.path.join(root, fname)
                # Ignore git markers and python cache
                if fname in ('COLCON_IGNORE', '.gitkeep') or fname.endswith('.pyc'):
                    continue
                checked_count += 1
                size = os.path.getsize(fpath)
                if size == 0:
                    zero_byte_files.append(fpath)

    print(f"Scanned {checked_count} source files across rover_ws, station_ws, scripts, firmware, and docs.")

    if zero_byte_files:
        print(f"  [FAIL] Detected {len(zero_byte_files)} zero-byte source files:")
        for zf in zero_byte_files:
            print(f"    - {zf}")
        sys.exit(1)
    else:
        print("  [PASS] All source files are populated and non-empty.")
        sys.exit(0)

if __name__ == '__main__':
    check_source_integrity()
