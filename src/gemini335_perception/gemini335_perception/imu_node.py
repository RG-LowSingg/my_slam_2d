#!/usr/bin/env python3
"""
imu_node.py — Gemini 335 IMU ROS2 Node

Publishes:
  - /gemini335/imu  (sensor_msgs/Imu)

Uses pyorbbecsdk to capture accelerometer and gyroscope data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import pyorbbecsdk as ob


class ImuNode(Node):
    """ROS2 node that publishes IMU data from Gemini 335."""

    def __init__(self):
        super().__init__('gemini335_imu_node')

        # Parameters
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('frame_id_imu', 'gemini335_imu_frame')
        self.declare_parameter('topic_imu', 'gemini335/imu')

        self.frame_id = self.get_parameter('frame_id_imu').value
        topic = self.get_parameter('topic_imu').value

        # Publisher
        self.pub_imu = self.create_publisher(Imu, topic, 50)

        # Camera state
        self.pipeline = None

        # Initialize
        self._init_imu()

        # Timer — IMU runs at higher rate than camera
        # Poll at 200Hz (5ms), actual rate depends on sensor config
        self.timer = self.create_timer(0.005, self._timer_callback)

        self.get_logger().info(f'IMU node started, publishing to {topic}')

    def _init_imu(self):
        """Initialize Orbbec pipeline for IMU-only streaming."""
        ctx = ob.Context()
        device_list = ctx.query_devices()
        if device_list.get_count() == 0:
            self.get_logger().error('No Orbbec device found!')
            raise RuntimeError('No Orbbec device found')

        device = device_list.get_device_by_index(0)
        info = device.get_device_info()
        self.get_logger().info(f'Device: {info.get_name()} (SN: {info.get_serial_number()})')

        # Check IMU sensors
        has_accel = False
        has_gyro = False
        try:
            if device.get_sensor(ob.OBSensorType.ACCEL_SENSOR) is not None:
                has_accel = True
        except Exception:
            pass
        try:
            if device.get_sensor(ob.OBSensorType.GYRO_SENSOR) is not None:
                has_gyro = True
        except Exception:
            pass

        if not has_accel and not has_gyro:
            self.get_logger().error('Device does not support IMU!')
            raise RuntimeError('No IMU sensors available')

        self.get_logger().info(f'IMU sensors: accel={has_accel}, gyro={has_gyro}')

        self.pipeline = ob.Pipeline(device)
        config = ob.Config()

        if has_accel:
            config.enable_accel_stream()
        if has_gyro:
            config.enable_gyro_stream()

        self.pipeline.start(config)
        self.has_accel = has_accel
        self.has_gyro = has_gyro

    def _timer_callback(self):
        """Poll IMU data and publish."""
        if self.pipeline is None:
            return

        try:
            frameset = self.pipeline.wait_for_frames(10)
        except Exception:
            return

        if frameset is None:
            return

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        has_data = False

        # Accelerometer
        if self.has_accel:
            try:
                accel_frame = frameset.get_frame_by_type(ob.OBFrameType.ACCEL_FRAME)
                if accel_frame is not None:
                    af = accel_frame.as_accel_frame()
                    imu_msg.linear_acceleration.x = float(af.get_x())
                    imu_msg.linear_acceleration.y = float(af.get_y())
                    imu_msg.linear_acceleration.z = float(af.get_z())
                    # Covariance unknown
                    imu_msg.linear_acceleration_covariance = [0.0] * 9
                    has_data = True
            except Exception:
                pass

        # Gyroscope
        if self.has_gyro:
            try:
                gyro_frame = frameset.get_frame_by_type(ob.OBFrameType.GYRO_FRAME)
                if gyro_frame is not None:
                    gf = gyro_frame.as_gyro_frame()
                    imu_msg.angular_velocity.x = float(gf.get_x())
                    imu_msg.angular_velocity.y = float(gf.get_y())
                    imu_msg.angular_velocity.z = float(gf.get_z())
                    imu_msg.angular_velocity_covariance = [0.0] * 9
                    has_data = True
            except Exception:
                pass

        # Orientation not provided by raw IMU
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8  # -1 = orientation unknown

        if has_data:
            self.pub_imu.publish(imu_msg)

    def destroy_node(self):
        """Clean shutdown."""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
