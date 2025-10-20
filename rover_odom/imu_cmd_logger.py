#!/usr/bin/env python3
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, AccelStamped
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ImuCmdAligner(Node):
    def __init__(self):
        super().__init__('imu_cmd_aligner')

        # Resolve a good default CSV directory inside the installed package share (…/share/rover_odom/tools/velocity_from_imu_identification)
        try:
            from ament_index_python.packages import get_package_share_directory
            default_csv_dir = os.path.join(get_package_share_directory('rover_odom'), 'velocity_from_imu_identification')
        except Exception:
            default_csv_dir = os.path.expanduser('~/.ros')

        os.makedirs(default_csv_dir, exist_ok=True)

        # Parameters
        self.declare_parameter('imu_topic', '/imu_odom')
        self.declare_parameter('cmd_topic', '/wheel_cmd')
        self.declare_parameter('output_topic', '/imu_cmd_aligned')
        self.declare_parameter('queue_size', 100)
        self.declare_parameter('sync_slop_s', 0.02)
        self.declare_parameter('write_csv', True)
        self.declare_parameter('csv_dir', default_csv_dir)
        self.declare_parameter('csv_basename', 'imu_cmd_aligned')
        self.declare_parameter('raw_accel_debug_mode', False) 

        imu_topic   = self.get_parameter('imu_topic').get_parameter_value().string_value
        cmd_topic   = self.get_parameter('cmd_topic').get_parameter_value().string_value
        output_topic= self.get_parameter('output_topic').get_parameter_value().string_value
        queue_size  = self.get_parameter('queue_size').get_parameter_value().integer_value
        sync_slop   = self.get_parameter('sync_slop_s').get_parameter_value().double_value
        self.write_csv = self.get_parameter('write_csv').get_parameter_value().bool_value
        csv_dir     = self.get_parameter('csv_dir').get_parameter_value().string_value
        csv_basename= self.get_parameter('csv_basename').get_parameter_value().string_value
        self.raw_accel_debug_mode = self.get_parameter('raw_accel_debug_mode').get_parameter_value().bool_value

        # Publisher
        self.pub = self.create_publisher(AccelStamped, output_topic, 10)

        # Subscribers + approx time sync
        self.sub_imu = Subscriber(self, Odometry, imu_topic, qos_profile=10)
        self.sub_cmd = Subscriber(self, Vector3Stamped, cmd_topic, qos_profile=10)
        self.sync = ApproximateTimeSynchronizer([self.sub_imu, self.sub_cmd], queue_size=queue_size, slop=sync_slop)
        self.sync.registerCallback(self.synced_cb)

        # CSV setup
        self.csv_writer = None
        self.csv_file = None
        if self.write_csv:
            os.makedirs(csv_dir, exist_ok=True)  # ensure directory exists
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(csv_dir, f"{csv_basename}_{stamp}.csv")
            self.get_logger().info(f"Will write aligned CSV to: {self.csv_path}")
        else:
            self.csv_path = None

        self.get_logger().info(
            f"ImuCmdAligner started. imu_topic={imu_topic}, cmd_topic={cmd_topic}, "
            f"output_topic={output_topic}, slop={sync_slop:.3f}s, "
            f"raw_accel_debug_mode={self.raw_accel_debug_mode}"
        )

    def synced_cb(self, imu_msg: Odometry, cmd_msg: Vector3Stamped):
        out = AccelStamped()
        out.header = Header()
        out.header.stamp = cmd_msg.header.stamp
        out.header.frame_id = imu_msg.header.frame_id or 'odom'

        out.accel.linear.x = imu_msg.twist.twist.linear.x
        out.accel.linear.y = imu_msg.twist.twist.linear.y
        out.accel.linear.z = imu_msg.twist.twist.linear.z

        out.accel.angular.x = cmd_msg.vector.x
        out.accel.angular.y = cmd_msg.vector.y
        out.accel.angular.z = cmd_msg.vector.z

        self.pub.publish(out)

        if self.write_csv:
            self._write_row(imu_msg, cmd_msg, out)

    def _write_row(self, imu_msg: Odometry, cmd_msg: Vector3Stamped, out_msg: AccelStamped):
        if self.csv_writer is None:
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Choose headers that match what odom is publishing into twist.linear
            if self.raw_accel_debug_mode:
                lin_labels = ['imu_ax', 'imu_ay', 'imu_az']
            else:
                lin_labels = ['imu_vx', 'imu_vy', 'imu_vz']

            self.csv_writer.writerow([
                't_out_sec','t_out_nanosec','t_imu_sec','t_imu_nanosec','t_cmd_sec','t_cmd_nanosec',
                *lin_labels, 'cmd_vx','cmd_vy','cmd_vz',
            ])

        self.csv_writer.writerow([
            out_msg.header.stamp.sec, out_msg.header.stamp.nanosec,
            imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec,
            cmd_msg.header.stamp.sec, cmd_msg.header.stamp.nanosec,
            imu_msg.twist.twist.linear.x, imu_msg.twist.twist.linear.y, imu_msg.twist.twist.linear.z,
            cmd_msg.vector.x, cmd_msg.vector.y, cmd_msg.vector.z,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            if self.csv_file:
                self.csv_file.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = ImuCmdAligner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
