import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32, Bool


class BehaviorPlannerNode(Node):
    def __init__(self):
        super().__init__('behavior_planner_node')
        self.get_logger().info('🧠 Davranış Planlayıcı (Beyin) Başlatıldı.')

        # === Subscribers ===
        self.create_subscription(Odometry, '/localization/odometry', self.odometry_callback, 10)
        self.create_subscription(PoseStamped, '/mission/current_goal', self.goal_callback, 10)
        self.create_subscription(Float32, '/front_distance', self.front_distance_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_detected_callback, 10)
        self.create_subscription(String, '/obstacle_type', self.obstacle_type_callback, 10)
        self.create_subscription(String, '/perception/traffic_light_status', self.traffic_light_callback, 10)
        self.create_subscription(Twist, '/pid/steering_cmd', self.pid_callback, 10)  # PID'den direksiyon/hız (opsiyonel)

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, '/vehicle_command', 10)
        self.state_pub = self.create_publisher(String, '/behavior/state', 10)
        self.obstacle_check_pub = self.create_publisher(Bool, '/behavior/enable_obstacle_check', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.override_pub = self.create_publisher(Bool, '/control/override_active', 10)  # <— HW’ye override sinyali

        # === State ===
        self.current_pose = None
        self.current_goal = None
        self.traffic_light = None
        self.behavior_state = "BEKLEMEDE"
        self.front_distance = 20.0
        self.obstacle_detected = False
        self.obstacle_type = "none"
        self.pid_cmd = None  # PID'den gelen Twist (speed, steering)

        # --- Statik sollama/sağlama param & durumları ---
        self.in_right_lane = True               # Başlangıçta sağ şerit varsay
        self.manual_overtake_active = False     # Script çalışıyor mu?
        self.script = []                        # [('GO_LEFT',2.5), ('STOP',0.5), ...]
        self.script_idx = 0
        self.phase_end_time = None

        # Süre ve hız paramları (görseldeki gibi)
        self.TRIGGER_DIST  = 7.0   # m – engel bu mesafeden yakınsa tetikle
        self.T_LEFT        = 6.0   # s – tam sola kır
        self.T_RIGHT       = 10.0   # s – tam sağa kır
        self.T_STOP        = 3   # s – kısa dur
        self.T_STRAIGHT    = 4   # s – düz toparlama
        self.V_MAN         = 0.35  # m/s – manevra hızı (yavaş)
        self.STEER_LEFT    = 1.0   # +sol (gerekirse işareti çevir)
        self.STEER_RIGHT   = -1.0  # -sağ
        self.is_stop = False       # Acil durumu kontrol için (log amaçlı)

        self.get_logger().info(
            f'⏱️ Statik manevra: L={self.T_LEFT}s, R={self.T_RIGHT}s, '
            f'STOP={self.T_STOP}s, STRAIGHT={self.T_STRAIGHT}s, v={self.V_MAN} m/s'
        )

        # Ana karar döngüsü
        self.create_timer(0.1, self.decision_loop)

    # === Callbacks ===
    def pid_callback(self, msg: Twist): self.pid_cmd = msg
    def odometry_callback(self, msg: Odometry): self.current_pose = msg.pose.pose
    def goal_callback(self, msg: PoseStamped): self.current_goal = msg.pose
    def front_distance_callback(self, msg: Float32): self.front_distance = msg.data
    def obstacle_detected_callback(self, msg: Bool): self.obstacle_detected = msg.data
    def obstacle_type_callback(self, msg: String): self.obstacle_type = msg.data
    def traffic_light_callback(self, msg: String): self.traffic_light = msg.data

    # === Helpers ===
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _override_on(self):
        self.override_pub.publish(Bool(data=True))

    def _override_off(self):
        self.override_pub.publish(Bool(data=False))

    def _start_script(self, started_from_right: bool):
        """Görseldeki gibi zaman tabanlı scripti oluştur ve çalıştır."""
        if started_from_right:
            # Önce 0.5s dur, sonra SOLA → DUR → SAĞA → DÜZ
            self.script = [
                ('STOP', self.T_STOP),          # PRE-STOP (0.5s)
                ('GO_LEFT', self.T_LEFT),
                ('STOP', self.T_STOP),
                ('GO_RIGHT', self.T_RIGHT),
                ('STRAIGHT', self.T_STRAIGHT)
            ]
        else:
            # Önce 0.5s dur, sonra SAĞA → DUR → SOLA → DÜZ
            self.script = [
                ('STOP', self.T_STOP),          # PRE-STOP (0.5s)
                ('GO_RIGHT', self.T_RIGHT),
                ('STOP', self.T_STOP),
                ('GO_LEFT', self.T_LEFT),
                ('STRAIGHT', self.T_STRAIGHT)
            ]

        self.script_idx = 0
        self.manual_overtake_active = True
        self.phase_end_time = self.now() + self.script[0][1]
        self.behavior_state = f"MAN_{self.script[0][0]}"
        self._override_on()  # <— HW override aç
        self.publish_behavior_state()
        self.get_logger().warn(f"▶️ Statik manevra başladı: {self.script}, {self.in_right_lane}")

    def _run_manual_overtake_step(self) -> bool:
        """
        Override scriptinin bir adımını yürütür.
        True → script devam ediyor; False → bitti.
        """
        name, _ = self.script[self.script_idx]

        # Faz bitti mi?
        if self.now() >= self.phase_end_time:
            if name == 'GO_LEFT':
                self.in_right_lane = False
            if name == 'GO_RIGHT':
                self.in_right_lane = False

            self.script_idx += 1
            if self.script_idx >= len(self.script):
                # Script tamam
                self.manual_overtake_active = False
                self.phase_end_time = None
                self.behavior_state = "NORMAL"
                self.publish_behavior_state()
                self._override_off()  # <— HW override kapat
                return False

            next_name, next_dur = self.script[self.script_idx]
            self.phase_end_time = self.now() + next_dur
            self.behavior_state = f"MAN_{next_name}"
            self.publish_behavior_state()
            name = next_name

        # Aktif faz komutu (override → PID yok say)
        if name == 'GO_LEFT':
            self.publish_vehicle_command(self.V_MAN, self.STEER_LEFT)
        elif name == 'GO_RIGHT':
            self.publish_vehicle_command(self.V_MAN, self.STEER_RIGHT)
        elif name == 'STOP':
            self.publish_vehicle_command(0.0, 0.0)
        elif name == 'STRAIGHT':
            self.publish_vehicle_command(self.V_MAN, 0.0)

        return True

    # === Karar Mekanizması ===
    def decision_loop(self):
        CRITICAL_DISTANCE = 1.0

        # 0) Acil durum & kırmızı ışık override
        if self.front_distance < CRITICAL_DISTANCE:
            self.behavior_state = "KRITIK_ENGEL_DUR"
            self.publish_vehicle_command(0.0, 0.0)
            self.publish_emergency_stop(True)
            self.publish_behavior_state()
            self.is_stop = True
            self.get_logger().warn("🚨 KRİTİK ENGEL DUR!")
            return
        else:
            self.publish_emergency_stop(False)

        if self.traffic_light == "RED":
            self.behavior_state = "DUR"
            self.publish_vehicle_command(0.0, 0.0)
            self.publish_behavior_state()
            return

        # 1) Override scripti aktifse: sadece onu yürüt ve çık
        if self.manual_overtake_active:
            if self._run_manual_overtake_step():
                return
            # bitti ise normal akışa düşer

        # 2) Engel var ve manevra henüz başlamadıysa → başlat
        if self.front_distance < 4.5:
            #self.get_logger().info("abii enegli bulduk laaa", self.in_right_lane)
            self.behavior_state = "STATIK_ENGELDE_BEKLE"
            if (self.front_distance < self.TRIGGER_DIST) and (not self.manual_overtake_active):
                self._start_script(started_from_right=self.in_right_lane)
                # ilk adımı hemen uygula
                self._run_manual_overtake_step()
                return
            self.publish_behavior_state()
            if not self.manual_overtake_active:
                self.publish_vehicle_command(0.0, 0.0)
            return

        # 3) NORMAL: override yokken şerit takibi/PID pas-through (istersen açık kalsın)
        self.behavior_state = "NORMAL"
        if self.pid_cmd is not None:
            self.publish_vehicle_command(self.pid_cmd.linear.x, self.pid_cmd.angular.z)
        else:
            # PID yoksa güvenli bekle
            self.publish_vehicle_command(0.0, 0.0)
        self.publish_behavior_state()

    # === Publishers ===
    def publish_vehicle_command(self, speed: float, steering: float):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steering
        self.cmd_pub.publish(msg)

    def publish_behavior_state(self):
        msg = String()
        msg.data = self.behavior_state
        self.state_pub.publish(msg)

    def publish_emergency_stop(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.emergency_stop_pub.publish(msg)


# === Main ===
def main(args=None):
    rclpy.init(args=args)
    node = BehaviorPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
