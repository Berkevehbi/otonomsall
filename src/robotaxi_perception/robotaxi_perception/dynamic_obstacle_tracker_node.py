#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, Bool, Float32
import sensor_msgs_py.point_cloud2 as pc2
import math
import cv2
import statistics


class DynamicObstacleTracker3D(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_tracker_3d')

        self.front_distance = 20.0
        self.frames_confirm = 3
        self._hit_count = 0

        # FOV (ör. 40° = ±20°)
        self.fov_deg = 25.0

        # --- Debug timer için paylaşılan durum ---
        self._dbg_has_points = False
        self._dbg_confirmed = False
        self._dbg_front_distance = self.front_distance

        # Publishers
        self.obstacle_type_pub = self.create_publisher(String, '/obstacle_type', 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/front_distance', 10)

        # Subscriptions
        self.create_subscription(PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        # self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.image_callback, 10)

        # --- 1 saniyede bir debug log ---
        self.create_timer(3.0, self._debug_timer_cb)

        self.get_logger().info("📡 Dinamik Engel Takipçisi 3D Lidar + ZED Kamera başlatıldı.")

    def pointcloud_callback(self, cloud_msg):
        front_points = []
        min_distance = 20.0  # varsayılan
        half_fov = self.fov_deg / 2.0

        # PointCloud2 verisini oku
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            # Ön bölge filtresi: ±1.5m yatay, ±1.5m dikey, ileri yönde
            if -1.5 <= y <= 1.5 and -1.5 <= z <= 1.5 and x > 0:
                # FOV filtresi
                bearing_deg = math.degrees(math.atan2(y, x))
                if abs(bearing_deg) > half_fov:
                    continue
                distance = math.sqrt(x**2 + y**2 + z**2)
                front_points.append(distance)

        if front_points:
            min_distance = statistics.median(front_points)

        self.front_distance = min_distance
        self.distance_pub.publish(Float32(data=self.front_distance))

        raw_detected = (len(front_points) > 0) and (self.front_distance < 6.0)
        if raw_detected:
            self._hit_count += 1
        else:
            self._hit_count = 0

        confirmed = (self._hit_count >= self.frames_confirm)
        self.obstacle_detected_pub.publish(Bool(data=confirmed))

        # --- Debug timer için durumu güncelle ---
        self._dbg_has_points = (len(front_points) > 0)
        self._dbg_confirmed = confirmed
        self._dbg_front_distance = self.front_distance

    def _debug_timer_cb(self):
        # Her 1 saniyede bir durum raporu
        if not self._dbg_has_points:
            self.get_logger().warn("⚠️ Ön bölgede nokta yok, varsayılan mesafe: 20.0 m")
            return

        self.get_logger().info(f"📏 Ön mesafe (3D): {self._dbg_front_distance:.2f} m")
        if self._dbg_confirmed:
            self.get_logger().warn(f"⛔ Engel ALGILANDI (3D) — {self.frames_confirm} ardışık frame teyit")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTracker3D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node elle durduruldu (CTRL+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()