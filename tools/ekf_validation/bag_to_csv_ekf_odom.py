#!/usr/bin/env python3
import os, sys, math, argparse, yaml, glob
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

try:
    import rosbag2_py
except Exception:
    sys.exit("rosbag2_py not found. Try: sudo apt install ros-jazzy-rosbag2*")

DEF_BAGS_DIR = os.path.expanduser('~/bags')
DEF_PREFIX   = 'ekf_drive_'
DEF_TOPIC    = '/ekf_odom'

def yaw_from_quat(qx,qy,qz,qw):
    s = 2.0*(qw*qz + qx*qy)
    c = 1.0 - 2.0*(qy*qy + qz*qz)
    return math.atan2(s,c)

def find_latest_bag_dir(bags_dir: str, prefix: str) -> str:
    """
    Return the latest bag directory under bags_dir whose basename starts with prefix.
    Chooses by lexicographic order of the timestamp suffix (works with YYYYMMDD_HHMMSS).
    """
    if not os.path.isdir(bags_dir):
        raise FileNotFoundError(f"Bags directory not found: {bags_dir}")
    # collect dirs matching prefix
    candidates = [d for d in glob.glob(os.path.join(bags_dir, prefix + '*')) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No bag directories like '{prefix}*' in {bags_dir}")
    # sort by basename descending (timestamped names sort lexicographically)
    latest = sorted(candidates, key=lambda p: os.path.basename(p), reverse=True)[0]
    return latest

def read_series(bag_dir: str, topic: str):
    meta = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta):
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")

    with open(meta, "r") as f:
        storage_id = yaml.safe_load(f).get("storage_identifier", "mcap")

    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    conv    = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise SystemExit(f"Topic '{topic}' not in bag. Available:\n" + "\n".join(sorted(topics)))

    rows = []
    t0 = None
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        rows.append((t - t0, x, y, yaw))

    if not rows:
        raise SystemExit(f"No messages on {topic} in {bag_dir}")
    return rows

def main():
    # Optional overrides, but all args are optional so you can run with none.
    ap = argparse.ArgumentParser(description="Export /ekf_odom from latest ekf_drive_* bag to CSV.")
    ap.add_argument("--bags_dir", default=DEF_BAGS_DIR, help="Root directory containing ekf_drive_* folders (default: ~/bags)")
    ap.add_argument("--prefix",   default=DEF_PREFIX,   help="Bag folder prefix (default: ekf_drive_)")
    ap.add_argument("--topic",    default=DEF_TOPIC,    help="Topic to export (default: /ekf_odom)")
    ap.add_argument("--bag",      default=None,         help="(Optional) explicit bag dir; if missing, picks latest ekf_drive_*")
    ap.add_argument("--out",      default=None,         help="(Optional) output CSV path; default: <bags_dir>/<bagname>.csv")
    args = ap.parse_args()

    # Resolve bag directory
    if args.bag:
        bag_dir = os.path.expanduser(args.bag.rstrip("/"))
    else:
        bag_dir = find_latest_bag_dir(os.path.expanduser(args.bags_dir), args.prefix)

    bag_base = os.path.basename(bag_dir.rstrip("/"))
    parent   = os.path.dirname(bag_dir.rstrip("/"))

    # Default output: sibling CSV with the same name as the bag folder
    out_csv = os.path.abspath(args.out) if args.out else os.path.join(parent, f"{bag_base}.csv")

    # Read and save
    rows = read_series(bag_dir, args.topic)
    arr = np.array(rows, dtype=float)
    # Ensure parent dir exists
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    np.savetxt(out_csv, arr, delimiter=",", header="t,x,y,yaw", comments="", fmt="%.9f")
    print(f"Wrote {out_csv} ({len(rows)} samples) from bag: {bag_dir}")

if __name__ == "__main__":
    main()
