#!/usr/bin/env python3
import os, sys, math, argparse, yaml
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

try:
    import rosbag2_py
except Exception as e:
    sys.exit("rosbag2_py not found. Try: sudo apt install ros-jazzy-rosbag2*")

def yaw_from_quat(qx,qy,qz,qw):
    s = 2.0*(qw*qz + qx*qy)
    c = 1.0 - 2.0*(qy*qy + qz*qz)
    return math.atan2(s,c)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to bag dir (contains metadata.yaml)")
    ap.add_argument("--topic", default="/ekf_odom")
    ap.add_argument("--out", default="ekf_odom.csv")
    args = ap.parse_args()

    bag = os.path.expanduser(args.bag.rstrip("/"))
    meta = os.path.join(bag, "metadata.yaml")
    if not os.path.exists(meta):
        sys.exit(f"metadata.yaml not found in {bag}")

    with open(meta,"r") as f:
        storage_id = yaml.safe_load(f).get("storage_identifier","mcap")

    storage = rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id)
    conv    = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        sys.exit(f"Topic '{args.topic}' not in bag. Available:\n" + "\n".join(sorted(topics)))

    rows = []
    t0 = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, Odometry)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        qx,qy,qz,qw = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        yaw = yaw_from_quat(qx,qy,qz,qw)
        t = t_ns * 1e-9
        if t0 is None: t0 = t
        rows.append((t - t0, x, y, yaw))

    if not rows:
        sys.exit(f"No messages on {args.topic}")

    out = os.path.abspath(args.out)
    np.savetxt(out, np.array(rows), delimiter=",", header="t,x,y,yaw", comments="")
    print(f"Wrote {out} ({len(rows)} samples)")

if __name__ == "__main__":
    main()
