#!/usr/bin/env python3
import socket
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField

HOST = '0.0.0.0'
PORT = 65432

class PiServer(Node):
    def __init__(self):
        super().__init__('pi_server')

        qos = QoSProfile(depth=10)

        self.imu_publisher_ = self.create_publisher(Imu, '/sensor/imu', qos)
        self.mag_publisher_ = self.create_publisher(MagneticField, '/sensor/mag', qos)

        self.gyr = None
        self.acc = None
        self.mag = None

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((HOST, PORT))
        self.s.listen()

        self.get_logger().info(f"Listening on {HOST}:{PORT}")
        self.conn, addr = self.s.accept()
        self.get_logger().info(f"Connected by {addr}")
        self.buffer = b""

        self.timer = self.create_timer(0.01, self.sensor_callback)
    
    def publish_imu(self, stamp):
        msg = Imu()
        msg.header.stamp = stamp 
        msg.header.frame_id = 'sensor_link'

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0]

        msg.angular_velocity.x = self.gyr[0]
        msg.angular_velocity.y = self.gyr[1]
        msg.angular_velocity.z = self.gyr[2]
        msg.angular_velocity_covariance = [0.0]* 9

        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_publisher_.publish(msg)

    def publish_mag(self, stamp):
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = 'sensor_link'

        msg.magnetic_field.x = self.mag[0]
        msg.magnetic_field.y = self.mag[1]
        msg.magnetic_field.z = self.mag[2]
        msg.magnetic_field_covariance = [0.0] * 9

        self.mag_publisher_.publish(msg)

    def sensor_callback(self):
        try:
            while True:
                data = self.conn.recv(4096)
                if not data:
                    print("[info] Connection closed by peer.")
                    break

                self.buffer += data

                while b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    values = list(map(float, line.decode().split(',')))

                    if len(values) != 9:
                        raise ValueError(f"Expected 9 comma-separated values, got {len(values)}")

                    gx, gy, gz = values[:3]
                    ax, ay, az = values[3:6]
                    mx, my, mz = values[6:]

                    self.gyr = np.array([gx, gy, gz])
                    self.acc = np.array([ax, ay, az])
                    self.mag = np.array([mx, -my, -mz])

                    stamp = self.get_clock().now().to_msg()
                    self.publish_imu(stamp)
                    self.publish_mag(stamp)

        finally:
            try:
                self.conn.close()
            except Exception:
                pass

            try:
                self.s.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    pi_server = PiServer()
    rclpy.spin(pi_server)
    pi_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()