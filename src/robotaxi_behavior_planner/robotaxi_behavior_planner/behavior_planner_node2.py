#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
        # PID passthrough (override kapalıyken)
        self.create_subscription(Twist, '/pid/steering_cmd', self.pid_callback, 10)

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, '/vehicle_command', 10)
        self.state_pub = self.create_publisher(String, '/behavior/state', 10)
        self.obstacle_check_pub = self.create_publisher(Bool, '/behavior/enable_obstacle_check', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.override_pub = self.create_publisher(Bool, '/control/override_active', 10)

        # === State ===
        self.current_pose = None
        self.current_goal = None
        self.traffic_light = None
        self.behavior_state = "BEKLEMEDE"

        self.front_distance = 20.0
        self.obstacle_detected = False
        self.obstacle_type = "none"  # "left" / "right" / "none"
        self.pid_cmd = None          # PID'den gelen Twist (speed, steering)

        # --- Statik manevra parametreleri ---
        self.TRIGGER_DIST   = 7.0    # m – bu mesafede scripti başlat
        self.T_LEFT         = 6.0    # s – tam sola kırma süresi
        self.T_RIGHT        = 10.0   # s – tam sağa kırma süresi
        self.T_STOP         = 3.0    # s – kısa dur
        self.T_STRAIGHT     = 2.0    # s – düz toparlama
        self.V_MAN          = 0.35   # m/s – manevra hızı (yavaş)
        self.STEER_LEFT     = 1.0    # +sol (gerekirse işareti çevir)
        self.STEER_RIGHT    = -1.0   # -sağ

        # --- Trim fazları (yeni) ---
        self.T_TRIM_RIGHT      = 10.0   # s – sola geçiş sonrası sağa küçük düzeltme
        self.STEER_TRIM_RIGHT  = -1.0  # sağa küçük direksiyon (tam sağ -1.0 ise)
        self.T_TRIM_LEFT       = 6.0   # s – sağa geçiş sonrası sola küçük düzeltme
        self.STEER_TRIM_LEFT   = 1.0   # sola küçük direksiyon (tam sol +1.0 ise)

        # --- Durum bayrakları ---
        self.in_right_lane = True            # başlangıçta sağ şerit
        self.manual_overtake_active = False  # zaman tabanlı script çalışıyor mu?
        self.script = []                     # [('GO_LEFT',2.5), ...]
        self.script_idx = 0
        self.phase_end_time = None
        self.overtake_count = 0              # tamamlanan manevra sayısı (1: solda kalındı, 2: sağda kalındı)

        self.is_stop = False  # acil durumu işareti (log için)

        self.get_logger().info(
            f'⏱️ Statik manevra param: L={self.T_LEFT}s, R={self.T_RIGHT}s, '
            f'STOP={self.T_STOP}s, STRAIGHT={self.T_STRAIGHT}s, v={self.V_MAN} m/s'
        )

        # Varsayılan: engel kontrol açık
        self.obstacle_check_pub.publish(Bool(data=True))

        # Ana karar döngüsü
        self.create_timer(0.1, self.decision_loop)

    # === Callbacks ===
    def pid_callback(self, msg: Twist):
        self.pid_cmd = msg

    def odometry_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg.pose

    def front_distance_callback(self, msg: Float32):
        self.front_distance = msg.data

    def obstacle_detected_callback(self, msg: Bool):
        self.obstacle_detected = msg.data

    def obstacle_type_callback(self, msg: String):
        # beklenti: "left" / "right" / "none"
        self.obstacle_type = (msg.data or "none").lower()

    def traffic_light_callback(self, msg: String):
        self.traffic_light = msg.data

    # === Helpers ===
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _override_on(self):
        self.override_pub.publish(Bool(data=True))

    def _override_off(self):
        self.override_pub.publish(Bool(data=False))

    def _start_script(self, target_lane: str, return_after: bool = False):
        """
        Hedef şeride zaman tabanlı geçişi başlatır.
        target_lane: 'left' veya 'right'
        return_after: True ise eski şeride geri dön (yarışma için False).
        """
        seq = [('STOP', self.T_STOP)]

        if target_lane == 'left':
            # Sola geç + ardından 3 sn sağa trim
            seq += [('GO_LEFT', self.T_LEFT),
                    ('TRIM_RIGHT', self.T_TRIM_RIGHT)]
            if return_after:
                seq += [('STOP', self.T_STOP), ('GO_RIGHT', self.T_RIGHT)]
        elif target_lane == 'right':
            # Sağa geç + ardından 3 sn sola trim (YENİ)
            seq += [('GO_RIGHT', self.T_RIGHT),
                    ('TRIM_LEFT', self.T_TRIM_LEFT)]
            if return_after:
                seq += [('STOP', self.T_STOP), ('GO_LEFT', self.T_LEFT)]
        else:
            # bilinmeyen hedef → güvenli tarafta kal
            self.get_logger().warn(f"⚠️ Bilinmeyen target_lane='{target_lane}', mevcutta kalınıyor.")

        # Her durumda toparlama ile bitir
        seq += [('STRAIGHT', self.T_STRAIGHT)]

        self.script = seq
        self.script_idx = 0
        self.manual_overtake_active = True
        self.phase_end_time = self.now() + self.script[0][1]
        self.behavior_state = f"MAN_{self.script[0][0]}"

        self._override_on()
        self.publish_behavior_state()
        self.get_logger().warn(f"▶️ Statik manevra başladı: {self.script}, in_right_lane={self.in_right_lane}")

    def _run_manual_overtake_step(self) -> bool:
        """
        Override scriptinin bir adımını yürütür.
        True → script devam ediyor; False → bitti.
        """
        name, _dur = self.script[self.script_idx]

        # Faz süresi doldu mu?
        if self.now() >= self.phase_end_time:
            # Şerit bayrağını lane-change fazlarının sonunda güncelle
            if name == 'GO_LEFT':
                self.in_right_lane = False
            elif name == 'GO_RIGHT':
                self.in_right_lane = True

            # Sonraki faza geç
            self.script_idx += 1
            if self.script_idx >= len(self.script):
                # Script tamamlandı
                self.manual_overtake_active = False
                self.phase_end_time = None
                self.behavior_state = "NORMAL"
                self.publish_behavior_state()
                self._override_off()
                self.overtake_count += 1
                self.get_logger().info(
                    f"✅ Manevra tamamlandı. overtake_count={self.overtake_count}, in_right_lane={self.in_right_lane}"
                )
                return False

            # Yeni faza hazırlık
            next_name, next_dur = self.script[self.script_idx]
            self.phase_end_time = self.now() + next_dur
            self.behavior_state = f"MAN_{next_name}"
            self.publish_behavior_state()
            name = next_name

        # Aktif faz komutları (override → PID yok sayılır)
        if name == 'GO_LEFT':
            self.publish_vehicle_command(self.V_MAN, self.STEER_LEFT)
        elif name == 'GO_RIGHT':
            self.publish_vehicle_command(self.V_MAN, self.STEER_RIGHT)
        elif name == 'TRIM_RIGHT':
            # Sola geçiş sonrası, yön/direksiyon düzeltmek için 3 sn sağa küçük kır
            self.publish_vehicle_command(self.V_MAN, self.STEER_TRIM_RIGHT)
        elif name == 'TRIM_LEFT':
            # Sağa geçiş sonrası, yön/direksiyon düzeltmek için 3 sn sola küçük kır (YENİ)
            self.publish_vehicle_command(self.V_MAN, self.STEER_TRIM_LEFT)
        elif name == 'STOP':
            self.publish_vehicle_command(0.0, 0.0)
        elif name == 'STRAIGHT':
            self.publish_vehicle_command(self.V_MAN, 0.0)

        return True

    def _choose_target_lane(self) -> str:
        """
        Engel tipine göre hedef şeridi seç; yoksa mevcut şeridin tersi.
        """
        if self.obstacle_type == "right":
            return "left"
        if self.obstacle_type == "left":
            return "right"
        # bilgi yoksa mevcut şeridin karşısına geç
        return "left" if self.in_right_lane else "right"

    # === Ana karar mekanizması ===
    def decision_loop(self):
        CRITICAL_DISTANCE = 1.0  # acil fren eşiği

        # 0) Acil durum
        if self.front_distance < CRITICAL_DISTANCE:
            self.behavior_state = "KRITIK_ENGEL_DUR"
            self.publish_vehicle_command(0.0, 0.0)
            self.publish_emergency_stop(True)
            self.publish_behavior_state()
            if not self.is_stop:
                self.get_logger().warn("🚨 KRİTİK ENGEL DUR!")
                self.is_stop = True
            return
        else:
            self.publish_emergency_stop(False)
            self.is_stop = False

        # Kırmızı ışık
        if (self.traffic_light or "").upper() == "RED":
            self.behavior_state = "DUR"
            self.publish_vehicle_command(0.0, 0.0)
            self.publish_behavior_state()
            return

        # 1) Override scripti aktifse sadece onu yürüt
        if self.manual_overtake_active:
            if self._run_manual_overtake_step():
                return
            # biterse NORMAL akışa düşer

        # 2) Engel tetikleyicisi
        # Yakın engel görünce güvenli bekle, TRIGGER_DIST içinde scripti başlat
        if self.front_distance < 4.5:
            self.get_logger().info(
                f"🧱 Engel tespit: dist={self.front_distance:.2f} m, in_right={self.in_right_lane}"
            )
            self.behavior_state = "STATIK_ENGELDE_BEKLE"

            if (self.front_distance < self.TRIGGER_DIST) and (not self.manual_overtake_active):
                # 1. manevra: SOL'a geç ve kal (trim_right içerir)
                if self.overtake_count == 0:
                    self._start_script(target_lane='left', return_after=False)
                # 2. manevra: SAĞ'a geç ve kal (trim_left içerir)
                elif self.overtake_count == 1:
                    self._start_script(target_lane='right', return_after=False)
                else:
                    # Sonrakiler: engel tarafına göre karar ver; yine kalıcı
                    self._start_script(self._choose_target_lane(), return_after=False)

                # İlk adımı hemen uygula
                self._run_manual_overtake_step()
                return

            self.publish_behavior_state()
            if not self.manual_overtake_active:
                self.publish_vehicle_command(0.0, 0.0)
            return

        # 3) NORMAL: override yokken PID passthrough
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
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)
        self.cmd_pub.publish(msg)

    def publish_behavior_state(self):
        msg = String()
        msg.data = self.behavior_state
        self.state_pub.publish(msg)

    def publish_emergency_stop(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
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
