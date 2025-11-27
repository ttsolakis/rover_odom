#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64   # <-- NEW

import smbus  # provided by python3-smbus (already installed)


class Bmm150Driver:
    """
    Minimal BMM150 I2C driver for raw X/Y/Z readings.

    - Uses I2C bus and address provided at init.
    - Does NOT apply full factory trim compensation (good enough for yaw + relative field).
    """

    # BMM150 registers (from datasheet)
    _REG_POWER_CTRL = 0x4B
    _REG_OPMODE = 0x4C
    _REG_DATA_X_LSB = 0x42  # X/Y/Z/RHALL start here

    def __init__(self, bus_id: int = 1, address: int = 0x13):
        self.bus_id = bus_id
        self.address = address
        self.bus = smbus.SMBus(self.bus_id)

    @staticmethod
    def _twos_complement(value: int, bits: int) -> int:
        """Convert unsigned integer to signed (two's complement)."""
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    def initialize(self):
        """
        Put BMM150 into normal mode at ~10 Hz ODR.

        Sequence (datasheet):
        - Set POWER_CTRL (0x4B) bit0 = 1 → from suspend to sleep
        - Set OPMODE (0x4C) op_mode bits = 00b → normal mode (default ODR ~10 Hz)
        """
        # Power control: bit0 = 1 (set whole byte to 0x01 for simplicity)
        self.bus.write_byte_data(self.address, self._REG_POWER_CTRL, 0x01)
        time.sleep(0.01)

        # Normal mode, default data rate (all other bits 0)
        self.bus.write_byte_data(self.address, self._REG_OPMODE, 0x00)
        time.sleep(0.01)

    def read_raw_xyz(self):
        """
        Read raw X/Y/Z counts from BMM150.

        The magnetic measurement registers are packed as:
        - X: 13-bit (X_MSB[7:0], X_LSB[7:3])
        - Y: 13-bit (Y_MSB[7:0], Y_LSB[7:3])
        - Z: 15-bit (Z_MSB[7:0], Z_LSB[7:1])
        """
        data = self.bus.read_i2c_block_data(self.address, self._REG_DATA_X_LSB, 8)

        x_lsb, x_msb = data[0], data[1]
        y_lsb, y_msb = data[2], data[3]
        z_lsb, z_msb = data[4], data[5]
        # rhall_lsb, rhall_msb = data[6], data[7]  # not used here

        # 13-bit X/Y
        raw_x = ((x_msb << 5) | (x_lsb >> 3)) & 0x1FFF
        raw_y = ((y_msb << 5) | (y_lsb >> 3)) & 0x1FFF

        # 15-bit Z
        raw_z = ((z_msb << 7) | (z_lsb >> 1)) & 0x7FFF

        x = self._twos_complement(raw_x, 13)
        y = self._twos_complement(raw_y, 13)
        z = self._twos_complement(raw_z, 15)

        return x, y, z


class MagnetometerReaderNode(Node):
    """
    ROS2 node that reads BMM150 via I2C and publishes:
      - sensor_msgs/MagneticField on "magnetic_field"
      - std_msgs/Float64 yaw (rad) on "magnetometer/yaw"

    Parameters:
      - i2c_bus (int): I2C bus index (default 1 → /dev/i2c-1)
      - i2c_address (int): I2C address (default 0x13)
      - frame_id (string): frame of the magnetometer (default "magnetometer_link")
      - publish_rate (double): Hz (default 20.0)
      - scale_lsb_to_tesla (double): scaling factor per LSB (default 0.3e-6)
      - yaw_offset_deg (double): static heading offset in degrees (default 0.0)
                                (to align sensor heading with robot heading)
    """

    def __init__(self):
        super().__init__('magnetometer_reader')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x13)
        self.declare_parameter('frame_id', 'magnetometer_link')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        # Typical BMM150 resolution ~0.3 µT/LSB → 0.3e-6 Tesla/LSB
        self.declare_parameter('scale_lsb_to_tesla', 0.3e-6)
        # Static yaw offset (e.g. to make yaw=0 when robot faces "forward")
        self.declare_parameter('yaw_offset_deg', 0.0)

        bus_id = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.scale_lsb_to_tesla = (self.get_parameter('scale_lsb_to_tesla').get_parameter_value().double_value)
        yaw_offset_deg = (self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)
        self.yaw_offset_rad = math.radians(yaw_offset_deg)

        self.get_logger().info(f'Initializing BMM150 on I2C bus={bus_id}, address=0x{address:02X}')

        try:
            self.driver = Bmm150Driver(bus_id=bus_id, address=address)
            self.driver.initialize()
            self.get_logger().info('BMM150 initialization complete.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BMM150: {e}')
            raise

        # Publishers
        self.mag_pub = self.create_publisher(MagneticField, 'magnetic_field', 10)
        self.yaw_pub = self.create_publisher(Float64, 'magnetometer/yaw', 10)

        # Timer
        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.timer_callback)

    @staticmethod
    def _normalize_angle_rad(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_callback(self):
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.frame_id

        try:
            x_raw, y_raw, z_raw = self.driver.read_raw_xyz()
        except OSError as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return

        # Convert to Tesla using a simple fixed scale.
        # For accurate values, you’d apply Bosch’s trim compensation here.
        s = self.scale_lsb_to_tesla
        bx = x_raw * s
        by = y_raw * s
        bz = z_raw * s

        mag_msg.magnetic_field.x = bx
        mag_msg.magnetic_field.y = by
        mag_msg.magnetic_field.z = bz

        # Unknown covariance for now
        mag_msg.magnetic_field_covariance = [0.0] * 9

        # Publish magnetic field
        self.mag_pub.publish(mag_msg)

        # --- Compute yaw (heading) from horizontal components ---
        # NOTE: this is NOT tilt-compensated; assumes sensor roughly level.
        yaw_rad = math.atan2(by, bx) + self.yaw_offset_rad
        yaw_rad = self._normalize_angle_rad(yaw_rad)

        yaw_msg = Float64()
        yaw_msg.data = yaw_rad   # radians
        # yaw_msg.data = (180/math.pi)*yaw_rad   # degrees (just for debugging)


        self.yaw_pub.publish(yaw_msg)

        # Optionally enable for debugging:
        # yaw_deg = math.degrees(yaw_rad)
        # self.get_logger().debug(
        #     f"Raw: X={x_raw},Y={y_raw},Z={z_raw}, yaw≈{yaw_deg:.1f}°"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
