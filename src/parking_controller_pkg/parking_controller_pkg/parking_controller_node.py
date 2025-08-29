import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Gerekli Mesaj Tipleri
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from smart_can_msgs.msg import rc_unittoOmux, AUTONOMOUS_HB_MotorControl
from visualization_msgs.msg import Marker
from diagnostic_msgs.msg import DiagnosticArray
from tf_transformations import euler_from_quaternion

# TF2 Kütüphaneleri
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import math
import numpy as np

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller_node')

        # --- Parametreleri Tanımla ve Al ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_park_width', 3.0),
                ('park_depth', 4.5),
                ('lidar_min_angle', -1.57),
                ('lidar_max_angle', 1.57),
                ('safety_distance', 0.5),
                ('lookahead_distance', 2.0),
                ('path_following_speed', 0.7),
                ('wheelbase', 1.86)
            ])
        self.MIN_PARK_WIDTH       = self.get_parameter('min_park_width').value
        self.PARK_DEPTH           = self.get_parameter('park_depth').value
        self.LIDAR_MIN_ANGLE      = self.get_parameter('lidar_min_angle').value
        self.LIDAR_MAX_ANGLE      = self.get_parameter('lidar_max_angle').value
        self.SAFETY_DISTANCE      = self.get_parameter('safety_distance').value
        self.LOOKAHEAD_DISTANCE   = self.get_parameter('lookahead_distance').value
        self.PATH_FOLLOWING_SPEED = self.get_parameter('path_following_speed').value
        self.WHEELBASE            = self.get_parameter('wheelbase').value

        # --- Durum Makinesi ---
        self.states = {
            0: "SEARCHING_FOR_SIGN",
            1: "SEARCHING_FOR_SPACE",
            2: "PLANNING_PATH",
            3: "FOLLOWING_PATH",
            4: "FINALIZING_PARK",
            5: "PARKED_AND_DONE",
           -1: "EMERGENCY_STOP"
        }
        self.current_state = 0
        info = f"Başlangıç Durumu: {self.states[self.current_state]}"
        self.get_logger().info(info)
        print(f"[Terminal] {info}")

        # --- Publisherlar ---
        self.pub_rc            = self.create_publisher(rc_unittoOmux, '/beemobs/rc_unittoOmux', 10)
        self.pub_brake_pedal   = self.create_publisher(Float32, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.pub_handbrake     = self.create_publisher(AUTONOMOUS_HB_MotorControl, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.pub_throttle      = self.create_publisher(Float32, '/beemobs/RC_THRT_DATA', 10)
        self.pub_steering      = self.create_publisher(Float32, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.target_pose_pub   = self.create_publisher(PoseStamped, '/huncar/parking_target_pose', 10)
        private_ns             = f"/{self.get_name()}"
        self.state_pub         = self.create_publisher(String, f'{private_ns}/parking_state', 10)
        self.spot_pub          = self.create_publisher(Marker, f'{private_ns}/parking_spot_marker', 10)

        # --- Subscriberlar ---
        self.scan_subscriber   = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.odom_subscriber   = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        self.diag_subscriber   = self.create_subscription(DiagnosticArray, '/diagnostics', self.diag_callback, 10)
        self.yolo_subscriber   = self.create_subscription(String, '/yolo_detector/detected_object', self.yolo_callback, 10)
        self.path_subscriber   = self.create_subscription(Path, '/huncar/planned_path', self.path_callback, 10)

        # --- TF2 Dinleyici ---
        self.tf_buffer         = Buffer()
        self.tf_listener       = TransformListener(self.tf_buffer, self)
        self.current_odom      = None
        self.planned_path      = None
        self.is_emergency      = False
        self.park_timer        = None

        # --- Ana Döngü ---
        self.create_timer(0.1, self.main_loop)
        ready = "ParkingControllerNode hazır, ana döngü başlatıldı."
        self.get_logger().info(ready)
        print(f"[Terminal] {ready}")

    def scan_callback(self, msg: LaserScan):
        if self.current_state == 1:
            self.find_parking_spot(msg)

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        if self.current_state == 3:
            self.control_parking()

    def diag_callback(self, msg: DiagnosticArray):
        for s in msg.status:
            if s.level >= 2:
                err = f"Acil durum: {s.message}";
                self.get_logger().error(err)
                print(f"[Terminal][HATA] {err}")
                self.is_emergency = True
                self.update_state(-1)
                return

    def yolo_callback(self, msg: String):
        if self.current_state == 0 and 'park' in msg.data.lower():
            info = "Park tabelası algılandı. Park alanı aranıyor..."
            self.get_logger().info(info)
            print(f"[Terminal] {info}")
            self.update_state(1)

    def path_callback(self, msg: Path):
        if self.current_state == 2:
            info = f"Yörünge alındı ({len(msg.poses)} nokta). Takip başlıyor."
            self.get_logger().info(info)
            print(f"[Terminal] {info}")
            self.planned_path = msg
            self.update_state(3)

    def main_loop(self):
        if self.is_emergency:
            self.send_vehicle_command(0.0, 0.0, 0, handbrake_action='PULL')
            return
        # Durum yayınla
        self.state_pub.publish(String(data=self.states[self.current_state]))
        print(f"[Terminal] Mevcut durum: {self.states[self.current_state]}")
        # Duruma göre komut gönder
        if self.current_state == 0:
            self.send_vehicle_command(1.5, 0.0, 1)
        elif self.current_state == 1:
            self.send_vehicle_command(0.8, 0.0, 1)
        elif self.current_state == 2:
            self.send_vehicle_command(0.0, 0.0, 0)
        # FOLLOWING_PATH ve FINALIZING_PARK durumları odom_callback üzerinden kontrol edilir
        elif self.current_state in [5, -1]:
            self.send_vehicle_command(0.0, 0.0, 0, handbrake_action='PULL')

    def update_state(self, new_state):
        if self.current_state != new_state:
            prev = self.states[self.current_state]
            nxt  = self.states[new_state]
            msg  = f"Durum değiştirildi: {prev} -> {nxt}"
            self.get_logger().info(msg)
            print(f"[Terminal] {msg}")
            # park timeout timer iptal ve yeni timer ayarı
            if self.park_timer:
                self.park_timer.cancel()
            if new_state == 3:
                self.park_timer = self.create_timer(180.0, self._park_timeout)
            self.current_state = new_state

    def find_parking_spot(self, scan_msg: LaserScan):
        ranges  = np.array(scan_msg.ranges)
        angles  = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        mask    = (angles > self.LIDAR_MIN_ANGLE) & (angles < self.LIDAR_MAX_ANGLE) & np.isfinite(ranges)
        x, y    = ranges[mask]*np.cos(angles[mask]), ranges[mask]*np.sin(angles[mask])
        idxs    = np.argsort(y)
        y_s, x_s= y[idxs], x[idxs]
        gaps    = np.where(np.diff(y_s)>self.MIN_PARK_WIDTH)[0]
        if not len(gaps):
            return
        i       = gaps[0]
        spot_x, spot_y = (x_s[i]+x_s[i+1])/2.0, (y_s[i]+y_s[i+1])/2.0
        print(f"[Terminal] Base_link koordinatları: x={spot_x:.2f}, y={spot_y:.2f}")
        # PoseStamped oluştur ve dönüştür
        in_pose = PoseStamped()
        in_pose.header.frame_id = 'base_link'
        in_pose.header.stamp = self.get_clock().now().to_msg()
        in_pose.pose.position.x = spot_x
        in_pose.pose.position.y = spot_y
        transform = self.tf_buffer.lookup_transform('odom','base_link',rclpy.time.Time())
        out_pose  = tf2_geometry_msgs.do_transform_pose(in_pose, transform)
        print(f"[Terminal] Odom koordinatları: x={out_pose.pose.position.x:.2f}, y={out_pose.pose.position.y:.2f}")
        info = "Park yeri bulundu, path planner'a gönderiliyor."
        self.get_logger().info(info)
        print(f"[Terminal] {info}")
        self.target_pose_pub.publish(out_pose)
        self.spot_pub.publish(Marker(pose=out_pose.pose, header=out_pose.header))
        self.update_state(2)

    def control_parking(self):
        dist = self.planned_path and math.hypot(
            self.planned_path.poses[-1].pose.position.x - self.current_odom.pose.pose.position.x,
            self.planned_path.poses[-1].pose.position.y - self.current_odom.pose.pose.position.y)
        self.get_logger().debug(f"Hedefe uzaklık: {dist:.2f} m")
        print(f"[Terminal] Hedefe uzaklık: {dist:.2f} m")
        if dist and dist<0.3:
            self.update_state(4)

    def _park_timeout(self):
        warn = "Park süresi aşıldı, görev iptal edildi."
        self.get_logger().warning(warn)
        print(f"[Terminal] {warn}")
        self.send_vehicle_command(0.0,0.0,0,handbrake_action='PULL')
        self.current_state = -1

    def send_vehicle_command(self, throttle, steering, gear, handbrake_action='NONE'):
        # RC komutları
        rc_msg = rc_unittoOmux()
        rc_msg.rc_ignition        = 1
        rc_msg.rc_selection_gear  = int(gear)
        self.pub_rc.publish(rc_msg)
        # Fren pedal kontrolü (varsayılan)
        brake_val = 1.0 if throttle==0.0 else 0.0
        self.pub_brake_pedal.publish(Float32(data=brake_val))
        # El freni
        if handbrake_action=='PULL':
            hb = AUTONOMOUS_HB_MotorControl()
            hb.autonomous_hb_mot_en  = 1
            hb.autonomous_hb_motor_pwm = 200
            hb.autonomous_hb_mot_state = 0
            self.pub_handbrake.publish(hb)
            print(f"[Terminal] El freni çekildi.")
            return
        # Gaz ve Direksiyon
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steering)))

    def destroy_node(self):
        if hasattr(self,'park_timer'):
            self.park_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        finish = "Düğüm kapatılıyor, araç durduruluyor."
        node.get_logger().info(finish)
        print(f"[Terminal] {finish}")
        node.send_vehicle_command(0.0,0.0,0,handbrake_action='PULL')
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
