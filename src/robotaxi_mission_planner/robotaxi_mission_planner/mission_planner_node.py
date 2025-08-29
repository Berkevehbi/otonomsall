import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
import argparse
import math 
from math import atan2, asin, degrees
from std_msgs.msg import String, Float64, Bool
from smart_can_msgs.msg import Feedbacksteeringangle as FeedbackSteeringAngle
# path_planner modülünüzü import edin
try:
    from robotaxi_mission_planner.path_planner import (
        Graph,
        gps_graph_creater,
        read_geojson_features,
        match_geojson_to_kml,
        create_full_route,
        cripto,
        haversine
    )
except ImportError:
    import sys, os

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from path_planner import (
        Graph,
        gps_graph_creater,
        read_geojson_features,
        match_geojson_to_kml,
        create_full_route,
        cripto,
        haversine
    )


class MissionPlannerNode(Node):
    def __init__(self, coords_dict, graph, route, threshold: float = 5.0):
        super().__init__('mission_planner')
        # Publisher: Bir sonraki hedefi gönder
        self.goal_pub = self.create_publisher(PoseStamped, 'mission/current_goal', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/zed2/zed_node/imu/data',
            self.imu_callback,
            10  # QoS history depth
        )
         # Subscriber: direksiyon açısı
        self.subscription_feedback = self.create_subscription(
            FeedbackSteeringAngle,
            '/beemobs/FeedbackSteeringAngle',
            self.feedback_callback,
            10
        )
        
        self.stop_publisher = self.create_publisher(
            Bool,
            '/is_stop',
            10
        )
        # GPS abonesi ve takip fonksiyonu aynı callback içinde
        self.publisher_ = self.create_publisher(String, '/target_direction', 10)
        self.publisher_feedback_yon= self.create_publisher(Bool, '/feedback_yon', 10)
        self.g = graph
        self.coords = coords_dict
        self.current_path = route
        self.next_index = 0
        self.current_gps = None
        self.threshold = threshold
        self.orientation_q = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.heading = 0.0
        # Abone ol ve callback içinde takip et
        self.create_subscription(NavSatFix, '/gnss', self.follow_path_with_gps, 10)

    def follow_path_with_gps(self, msg: NavSatFix):
        """
        GPS mesajı geldiğinde:
        1) current_gps güncellenir ve loglanır,
        2) mevcut hedefe mesafe hesaplanır,
        3) eşik içinde ise next_index artırılır ve loglanır,
        4) ardından mevcut ya da yeni hedef yayınlanır.
        """
        # 1) Pozisyon güncelle
        self.current_gps = (msg.latitude, msg.longitude)
        self.get_logger().info(f"[GPS] lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}")

        # 2) Hedef dizisi bitmediyse mesafe hesapla
        if self.next_index < len(self.current_path):
            target_name = self.current_path[self.next_index]
            lat_goal, lon_goal = self.coords[target_name]
            dist = haversine(self.current_gps, (lat_goal, lon_goal))
            self.get_logger().info(f"Distance to {target_name}: {dist:.1f} m")
            # 3) Eşik içinde ise bir sonraki hedefe geç
            if dist <= self.threshold:
                self.get_logger().info(f"Reached {target_name}, moving to next")
                self.next_index += 1
                if self.next_index >= len(self.current_path):
                    return
                target_name = self.current_path[self.next_index]
            # 4) Hedefi yayınla
            lat, lon = self.coords[target_name]
            message_text = target_name.strip().lower()
            bool_msg = Bool()
            if "stop" in message_text:
                bool_msg.data = True
                self.get_logger().info(f"'stop' detected in message: '{msg.data}' → Publishing True")
            else:
                bool_msg.data = False
                self.get_logger().info(f"No 'stop' in message' → Publishing False")

           
            angle,bearing,rotatedbearing=self.calculate_angle_to_target(
                msg.latitude, msg.longitude, lat, lon, self.heading
            )
            self.stop_publisher.publish(bool_msg)
            self.publisher_direction_and_way(angle)
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = lon
            goal.pose.position.y = lat
            self.goal_pub.publish(goal)
            self.get_logger().info(f"Published goal: {target_name}")
    def calculate_angle_to_target(self,lat1, lon1, lat2, lon2, heading, rotation_angle=130):
        bearing_to_target = self.calculator(lat1, lon1, lat2, lon2)
        rotated_bearing = (bearing_to_target + rotation_angle) % 360
        angle = (rotated_bearing - heading + 360) % 360 
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
        self.get_logger().info(f"Calculated angle: {angle:.2f}°")
        return angle, bearing_to_target, rotated_bearing

    def calculator(self,lat1, lon1, lat2, lon2):
        lat1_rad = self.degrees_to_radians(lat1)
        lon1_rad = self.degrees_to_radians(lon1)
        lat2_rad = self.degrees_to_radians(lat2)
        lon2_rad = self.degrees_to_radians(lon2)

        delta_lon = lon2_rad - lon1_rad
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = (self.radians_to_degrees(bearing_rad) + 360) % 360

        return bearing_deg

    def degrees_to_radians(self, degrees):
        return degrees * math.pi / 180

    def radians_to_degrees(self, radians):
        return radians * 180 / math.pi
    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        return roll, pitch, yaw
    def feedback_callback(self, msg: FeedbackSteeringAngle):
        self.current_angle = msg.feedbacksteeringangle
        self.get_logger().info(f"Geri bildirim direksiyon açısı: {self.current_angle:.2f}°")
    def current_heading(self,yaw):
        yaw_degrees = degrees(yaw)
        if yaw_degrees < 0:
            yaw_degrees += 360  # Pozitif açıya çevirme
        return yaw_degrees

    def imu_callback(self,data):
        self.orientation_q = data.orientation
        # Quaternion'u Euler açısına dönüştür
        (roll, pitch, yaw) = self.euler_from_quaternion(self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w)
        self.heading = self.current_heading(yaw)
        #rospy.loginfo(f"{self.currentDegree}")
    def publisher_direction_and_way(self, angle):
        direction = ""
        if -35 <= angle <= 35:
            direction = "On"
        elif angle > 0:
            direction = "right"
        else:
            direction = "left"

        # Yönü ve açıyı yayınla
        msg = String()
        msg.data = f"{direction} ({angle:.2f}°)"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Way {msg.data}")

        # Feedback yönünü yayınla
        feedback_msg = Bool()
        feedback_msg.data = (direction != "On")
        self.publisher_feedback_yon.publish(feedback_msg)

def main():
    parser = argparse.ArgumentParser(
        description='Mission Planner Node: GeoJSON & KML based routing with GPS follow'
    )
    parser.add_argument('geojson_file', help='GeoJSON file path')

    args, unknown = parser.parse_known_args()
    
    rclpy.init()

    # Graph ve koordinatlar
    graph = Graph()
    KML_PATH = "/root/robotaxi_ws/src/robotaxi_mission_planner/robotaxi_mission_planner/Coordinates (1).kml"
    coords_dict = gps_graph_creater(graph, graph._VertexDict, kml_file_path=KML_PATH)

    # GeoJSON’dan rota elde et
    data = read_geojson_features(args.geojson_file)
    merged = match_geojson_to_kml(data, coords_dict)
    print(merged)

    full_route = create_full_route(graph, merged)

    # Node oluştur
    node = MissionPlannerNode(coords_dict, graph, full_route, threshold=5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()