#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        # 1) Publisher: filtrelenmiş odometri
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 10)

        # 2) Sensör abonelikleri
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu,       '/zed/zed_node/imu/data', self.imu_callback, 10)

        # 3) Durum: en son gelen veriler
        self.latest_gps = None
        self.latest_imu = None

        # 4) Döngü (20 Hz)
        self.create_timer(1.0 / 20.0, self.control_loop)

    def gps_callback(self, msg: NavSatFix):
        """GPS pozisyon ölçümünü alır."""
        self.latest_gps = msg
        self.get_logger().info(f"[GPS] lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

    def imu_callback(self, msg: Imu):
        """IMU açısal hız verisini alır."""
        self.latest_imu = msg
        self.get_logger().info(f"[IMU] ω_z={msg.angular_velocity.z:.3f}")

    def control_loop(self):
        """GPS ve IMU geldiyse odometri yayınlar."""
        if self.latest_gps is None or self.latest_imu is None:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # Pozisyonu GPS'ten al
        odom.pose.pose.position.x = self.latest_gps.latitude
        odom.pose.pose.position.y = self.latest_gps.longitude
        odom.pose.pose.position.z = self.latest_gps.altitude

        # Yönelimi IMU'dan al (burada yaw açısal hızı olarak kullanıyoruz)
        yaw = self.latest_imu.angular_velocity.z
        q = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Hızı sıfır, açısal hızı yaw olarak koy
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = yaw

        self.odom_pub.publish(odom)
        self.get_logger().info('Published filtered Odometry')

def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
