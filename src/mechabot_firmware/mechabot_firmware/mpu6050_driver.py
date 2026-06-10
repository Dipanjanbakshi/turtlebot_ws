#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import smbus
import math
from collections import deque

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

DEVICE_ADDRESS = 0x68


class MPU6050Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        # I2C
        self.bus = smbus.SMBus(1)
        self.init_i2c()

        # ROS Publisher
        self.publisher = self.create_publisher(
            Imu,
            "/imu/out",
            qos_profile=qos_profile_sensor_data
        )

        # Timer (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # IMU message
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "base_link"

        # Moving average buffers
        window = 5
        self.acc_x_buf = deque(maxlen=window)
        self.acc_y_buf = deque(maxlen=window)
        self.acc_z_buf = deque(maxlen=window)

        self.gyro_x_buf = deque(maxlen=window)
        self.gyro_y_buf = deque(maxlen=window)
        self.gyro_z_buf = deque(maxlen=window)

        # Orientation estimation
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("MPU6050 Driver Started")

    def moving_average(self, buffer, value):
        buffer.append(value)
        return sum(buffer) / len(buffer)

    def timer_callback(self):

        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)

        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)

        # Moving average
        acc_x = self.moving_average(self.acc_x_buf, acc_x)
        acc_y = self.moving_average(self.acc_y_buf, acc_y)
        acc_z = self.moving_average(self.acc_z_buf, acc_z)

        gyro_x = self.moving_average(self.gyro_x_buf, gyro_x)
        gyro_y = self.moving_average(self.gyro_y_buf, gyro_y)
        gyro_z = self.moving_average(self.gyro_z_buf, gyro_z)

        # Convert accelerometer (m/s^2)
        ax = (acc_x / 16384.0) * 9.81
        ay = (acc_y / 16384.0) * 9.81
        az = (acc_z / 16384.0) * 9.81

        # Convert gyro (rad/s)
        gx = (gyro_x / 131.0) * math.pi / 180.0
        gy = (gyro_y / 131.0) * math.pi / 180.0
        gz = (gyro_z / 131.0) * math.pi / 180.0

        # Time step
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate yaw
        self.yaw += gz * dt

        # Quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Fill IMU message
        self.imu_msg.header.stamp = now.to_msg()

        self.imu_msg.linear_acceleration.x = ax
        self.imu_msg.linear_acceleration.y = ay
        self.imu_msg.linear_acceleration.z = az

        self.imu_msg.angular_velocity.x = gx
        self.imu_msg.angular_velocity.y = gy
        self.imu_msg.angular_velocity.z = gz

        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = qz
        self.imu_msg.orientation.w = qw

        self.publisher.publish(self.imu_msg)

    def init_i2c(self):

        self.bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
        self.bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
        self.bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
        self.bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0)
        self.bus.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)

        self.get_logger().info("MPU6050 Connected")

    def read_raw_data(self, addr):

        high = self.bus.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus.read_byte_data(DEVICE_ADDRESS, addr + 1)

        value = (high << 8) | low

        if value > 32768:
            value -= 65536

        return value


def main():

    rclpy.init()
    node = MPU6050Driver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()