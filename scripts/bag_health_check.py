#!/usr/bin/env python3
"""Flag bags missing data a recording session was supposed to capture.

`ros2 bag record -a` (scripts/start.bash) records every topic dynamically
discovered at record time, so a topic showing 0 messages means whatever was
supposed to publish it either wasn't running or wasn't producing output —
not a recording-config problem. This exists because that went unnoticed for
every bag recorded before the camera/CV pipeline was wired up: run this
right after a test drive instead of finding out months later.

Usage:
    python3 scripts/bag_health_check.py                 # every bags/* dir
    python3 scripts/bag_health_check.py bags/20260519_150616
"""
import argparse
import glob
import os
import sys

import yaml

# Topics a normal autonomous test drive should produce non-zero messages
# for. Extend this list as new subsystems come online.
EXPECTED_NONZERO = [
    "/odom", "/gps", "/imu", "/cmd_drive", "/mpc/status",
    "/e_comms/kart_speed_m_per_s",
]
# Camera/CV specifically — called out separately since a missing camera is
# the failure mode this script exists to catch (see docs/rl_residual_plan.md).
CAMERA_TOPICS = ["/track_angles", "/camera/image_raw", "/safety/status"]


def load_topic_counts(bag_dir: str) -> dict:
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    with open(meta_path) as f:
        meta = yaml.safe_load(f)
    info = meta.get("rosbag2_bagfile_information", meta)
    counts = {}
    for entry in info.get("topics_with_message_count", []):
        tm = entry["topic_metadata"]
        counts[tm["name"]] = entry["message_count"]
    return counts


def check_bag(bag_dir: str) -> list:
    """Returns a list of warning strings; empty means healthy."""
    warnings = []
    try:
        counts = load_topic_counts(bag_dir)
    except (OSError, yaml.YAMLError) as e:
        return [f"could not read metadata.yaml: {e}"]

    for topic in EXPECTED_NONZERO:
        if counts.get(topic, 0) == 0:
            warnings.append(f"{topic}: 0 messages (expected data)")

    camera_present = any(counts.get(t, 0) > 0 for t in CAMERA_TOPICS)
    if not camera_present:
        seen = [t for t in CAMERA_TOPICS if t in counts]
        if seen:
            warnings.append(
                f"camera/CV topics present but empty ({', '.join(seen)}) — "
                "camera_node/opencv_pathfinder_node likely wasn't running"
            )
        else:
            warnings.append(
                "no camera/CV topics recorded at all — "
                "camera_node likely wasn't launched"
            )
    return warnings


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dirs", nargs="*",
                     help="bag directories to check (default: every bags/*)")
    args = ap.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bag_dirs = args.bag_dirs or sorted(
        d for d in glob.glob(os.path.join(repo_root, "bags", "*"))
        if os.path.isdir(d)
    )

    if not bag_dirs:
        print("no bags found")
        return 0

    any_warnings = False
    for bag_dir in bag_dirs:
        warnings = check_bag(bag_dir)
        name = os.path.basename(bag_dir.rstrip("/"))
        if warnings:
            any_warnings = True
            print(f"[WARN] {name}")
            for w in warnings:
                print(f"    - {w}")
        else:
            print(f"[ OK ] {name}")

    return 1 if any_warnings else 0


if __name__ == "__main__":
    sys.exit(main())
