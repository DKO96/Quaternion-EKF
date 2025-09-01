#!/usr/bin/env python3
import socket
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, MagneticField

from ahrs.filters import EKF
from ahrs.common.orientation import am2q, acc2q

HOST = '0.0.0.0'
PORT = 65432

class AHRSEKF(Node):
    def __init__(self):
        super().__init__('ahrs_ekf')

        self.imu_publisher_ = self.create_publisher(Imu, '/sensor/imu_data', 10)
        self.mag_publisher_ = self.create_publisher(MagneticField, '/sensor/mag_data', 10)

        self.gyr = None
        self.acc = None
        self.mag = None
        self.init = False
        self.ekf = EKF(mag=np.zeros(3), frame='NED')
        self.q = None
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.sensor_callback)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((HOST, PORT))
        self.s.listen()

        self.get_logger().info(f"Listening on {HOST}:{PORT}")
        self.conn, addr = self.s.accept()
        self.get_logger().info(f"Connected by {addr}")
        self.buffer = b""
    
    def publish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'sensor_link'

        t.transform.rotation.w = float(self.q[0])
        t.transform.rotation.x = float(self.q[1])
        t.transform.rotation.y = float(self.q[2])
        t.transform.rotation.z = float(self.q[3])

        self.tf_broadcaster.sendTransform(t)
    
    def ahrs_ekf(self):
        if not self.init:
            self.q = am2q(a=self.acc, m=self.mag, frame='NED')
            self.init = True
            return

        self.q = self.ekf.update(q=self.q, gyr=self.gyr, acc=self.acc, mag=self.mag)
        print(f"q: {self.q}")

        # self.publish_transform()
    
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
                    self.ahrs_ekf()


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
    ahrs_ekf = AHRSEKF()
    rclpy.spin(ahrs_ekf)
    ahrs_ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()