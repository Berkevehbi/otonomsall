import rclpy
from rclpy.node import Node
import time
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool

from smart_can_msgs.msg import (
    Rcunittoomux,
    Rcthrtdata,
    Autonomousbrakepedalcontrol,
    Autonomoussteeringmotcontrol,
    Autonomoushbmotorcontrol,
    Fbvehiclespeed,
    Feedbacksteeringangle
)


class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hw_interface_node')
        self.get_logger().info('🚗 Geliştirilmiş Donanım Arayüz Düğümü Başlatıldı.')

        # Steering target/feedback
        self.target_steering_angle = 0.0
        self.current_steering_angle = 0.0
        self.last_log_time = 0.0

        # === State ===
        self.current_speed = 0.0        # /vehicle_command hız
        self.current_steer = 0.0        # /vehicle_command açısal hız (rad/s) — sadece log
        self.systems_active = False     # Fren/direksiyon motorları aktif mi?
        self.sequence_ready = False     # Başlatma sekansı tamam mı?
        self.emergency_active = False   # Acil durdurma aktif mi?
        self.override_active = False    # Behavior override modu aktif mi?

        # Timeout (komut akışı kesilirse güvenli dur)
        self.last_cmd_time = time.time()
        self.cmd_timeout_s = 0.3

        # Twist.z -> hedef açı (deg) dönüşümü için nominal/override ayarları
        self.NOMINAL_STEER_GAIN = 16
        self.NOMINAL_STEER_MIN  = -8.0
        self.NOMINAL_STEER_MAX  = 10.0

        self.OVERRIDE_STEER_GAIN = 18.0   # istersen 20.0 yap
        self.OVERRIDE_STEER_MIN  = -15.0  # aracın gerçek min'i
        self.OVERRIDE_STEER_MAX  =  25.0  # aracın gerçek max'ı
        self.STEER_SIGN = -1.0  # yön tersse +1.0 yap

        self.debug_last_log_time = 0.0
        self.cmd_last_log_time = 0.0

        # === Subscribers ===
        self.create_subscription(Twist, '/vehicle_command', self.vehicle_command_callback, 10)

        self.create_subscription(
            Fbvehiclespeed, '/beemobs/FB_VehicleSpeed', self.can_speed_callback, 10)

        self.create_subscription(
            Feedbacksteeringangle, '/beemobs/FeedbackSteeringAngle', self.feedback_steering_callback, 10)

        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        # Override sinyali (Behavior’dan)
        self.create_subscription(Bool, '/control/override_active', self.override_callback, 10)

        # === Publishers ===
        self.vehicle_speed_publisher = self.create_publisher(Fbvehiclespeed, '/vehicle/feedback/speed', 10)
        self.rc_unit_publisher = self.create_publisher(Rcunittoomux, '/beemobs/rc_unittoOmux', 10)
        self.throttle_publisher = self.create_publisher(Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.steering_publisher = self.create_publisher(Autonomoussteeringmotcontrol, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.steer_target_pub = self.create_publisher(Float32, '/beemobs/steering_target_value', 10)
        self.brake_publisher = self.create_publisher(Autonomousbrakepedalcontrol, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.handbrake_publisher = self.create_publisher(Autonomoushbmotorcontrol, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)

        # === Timers ===
        self.periodic_control_timer = self.create_timer(0.05, self.periodic_publish)  # 50 Hz
        self.startup_timer = self.create_timer(2.0, self.start_vehicle_sequence)

        # İç durum (PWM flip/flop mantığı için)
        self.is_left_turned = False
        self.is_right_turned = False

        self.get_logger().info("Düğüm hazır. 2 saniye içinde otomatik başlatma sekansı başlayacak...")

    # ====== Subscribers callbacks ======
    def can_speed_callback(self, msg: Fbvehiclespeed):
        self.vehicle_speed_publisher.publish(msg)

    def feedback_steering_callback(self, msg: Feedbacksteeringangle):
        self.current_steering_angle = msg.feedbacksteeringangle

    def override_callback(self, msg: Bool):
        self.override_active = bool(msg.data)
        if self.override_active:
            self.get_logger().warn("OVERRIDE: Behavior manevra modu AKTİF. Şerit takibi/PID komutları yok sayılıyor.")
        else:
            self.get_logger().info("OVERRIDE: KAPALI. Normal moda dönüldü.")

    def vehicle_command_callback(self, msg: Twist):
        """ Gelen anlık hız ve direksiyon komutunu değişkenlere kaydeder. """
        self.last_cmd_time = time.time()

        # Mod'a göre kazanç ve limitler
        if self.override_active:
            k   = self.OVERRIDE_STEER_GAIN
            mn  = self.OVERRIDE_STEER_MIN
            mx  = self.OVERRIDE_STEER_MAX
            mode = "OVR"
        else:
            k   = self.NOMINAL_STEER_GAIN
            mn  = self.NOMINAL_STEER_MIN
            mx  = self.NOMINAL_STEER_MAX
            mode = "NOM"

        # Twist.z -> derece + saturasyon
        raw_deg   = msg.angular.z * k
        target_deg = self.STEER_SIGN * float(np.clip(raw_deg, mn, mx))

        now = time.time()
        if now - self.cmd_last_log_time >= 2.0:
            self.get_logger().info(
                f"[{mode}] v={msg.linear.x:.2f} m/s, steer_cmd={msg.angular.z:.2f} "
                f"→ raw={raw_deg:.1f}°, hedef={target_deg:.1f}° (lim[{mn},{mx}] k={k})"
            )
            self.cmd_last_log_time = now

        # Burası değiştirilecek rot balans bozulma durumunda
        self.target_steering_angle = target_deg
        self.current_speed = float(msg.linear.x)
        self.current_steer = float(msg.angular.z)
        self.steer_target_pub.publish(Float32(data=self.target_steering_angle))

    def emergency_stop_callback(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.get_logger().warn("‼️ ACİL DURDURMA AKTİF EDİLDİ! Araç durduruluyor.")
            self.apply_emergency_stop()
            self.emergency_active = True
        elif not msg.data and self.emergency_active:
            self.get_logger().info("🟢 Acil durdurma SONA ERDİ. Normal moda dönüldü.")
            self.release_emergency_stop()
            self.emergency_active = False

    # ====== Vehicle bring-up ======
    def start_vehicle_sequence(self):
        """Aracı sürüşe hazırlayan otomatik başlatma sekansı (bir kez)."""
        self.get_logger().info(">>> OTOMATİK BAŞLATMA SEKANSI BAŞLADI <<<")

        # Direksiyon motoru
        steering_enable_msg = Autonomoussteeringmotcontrol()
        steering_enable_msg.autonomous_steeringmot_en = 1
        self.steering_publisher.publish(steering_enable_msg)

        # Fren motoru
        brake_enable_msg = Autonomousbrakepedalcontrol()
        brake_enable_msg.autonomous_brakepedalmotor_en = 1
        brake_enable_msg.autonomous_brakemotor_voltage = 1
        brake_enable_msg.autonomous_brakepedalmotor_acc = 10000
        self.brake_publisher.publish(brake_enable_msg)
        self.systems_active = True
        self.get_logger().info("--> Motorlar aktif.")
        time.sleep(1.0)

        # Ateşleme ve vites
        self.get_logger().info("Ateşleme açılıyor ve vites N...")
        ignition_msg = Rcunittoomux()
        ignition_msg.rc_ignition = 1
        ignition_msg.rc_selectiongear = 0 # N
        self.rc_unit_publisher.publish(ignition_msg)
        time.sleep(1.5)

        # El freni indir
        self.get_logger().info("El freni indiriliyor...")
        hb_msg = Autonomoushbmotorcontrol()
        hb_msg.autonomous_hb_moten = 1
        hb_msg.autonomous_hb_motstate = 1 # indir
        hb_msg.autonomous_hb_motor_pwm = 200
        self.handbrake_publisher.publish(hb_msg)
        time.sleep(1.5)

        # Drive
        self.get_logger().info("Vites DRIVE...")
        drive_msg = Rcunittoomux()
        drive_msg.rc_ignition = 1
        drive_msg.rc_selectiongear = 1 # D
        self.rc_unit_publisher.publish(drive_msg)

        self.sequence_ready = True
        self.get_logger().info(">>> ✅ BAŞLATMA TAMAMLANDI! ARAÇ SÜRÜŞE HAZIR. <<<")
        self.startup_timer.cancel()

    # ====== Emergency ======
    def apply_emergency_stop(self):
        brake_msg = Autonomousbrakepedalcontrol()
        brake_msg.autonomous_brakepedalmotor_en = 1
        brake_msg.autonomous_brakemotor_voltage = 1
        brake_msg.autonomous_brakepedalmotor_acc = 10000
        brake_msg.autonomous_brakepedalmotor_per = 100
        self.brake_publisher.publish(brake_msg)

        throttle_msg = Rcthrtdata()
        throttle_msg.rc_thrt_pedal_press = 1
        throttle_msg.rc_thrt_pedal_position = 0
        self.throttle_publisher.publish(throttle_msg)

        hb_msg = Autonomoushbmotorcontrol()
        hb_msg.autonomous_hb_moten = 1
        hb_msg.autonomous_hb_motstate = 0  # çek
        hb_msg.autonomous_hb_motor_pwm = 200
        self.handbrake_publisher.publish(hb_msg)

        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1
        steering_msg.autonomous_steeringmot_pwm = 128
        # self.steering_publisher.publish(steering_msg)  # nötr PWM göndermek istersen aç

        self.target_steering_angle = 0.0
        self.steer_target_pub.publish(Float32(data=0.0))

    def release_emergency_stop(self):
        hb_msg = Autonomoushbmotorcontrol()
        hb_msg.autonomous_hb_moten = 1
        hb_msg.autonomous_hb_motstate = 1  # bırak
        hb_msg.autonomous_hb_motor_pwm = 200
        self.handbrake_publisher.publish(hb_msg)

    # ====== Periodic control loop ======
    def periodic_publish(self):
        """50Hz ana döngü: override varsa override fonksiyonu, yoksa normal akış."""
        if self.emergency_active:
            return
        if not self.systems_active or not self.sequence_ready:
            return

        # Komut time-out: komut akışı kesilirse güvenli dur
        if time.time() - self.last_cmd_time > self.cmd_timeout_s:
            self.current_speed = 0.0
            self.target_steering_angle = 0.0

        if self.override_active:
            self.periodic_publish_override()
            return

        # --- Normal mod (şerit takibi/PID tarafı) ---
        current_angle = self.current_steering_angle
        if self.target_steering_angle > current_angle + 2:
            self.handle_steering(1.0)
        elif self.target_steering_angle < current_angle - 2:
            self.handle_steering(-1.0)
        else:
            self.handle_steering(0.0)

        self.handle_speed(self.current_speed)

    def periodic_publish_override(self):
        """
        Behavior override aktifken çalışan ayrı döngü.
        Şerit takibi/PID’den gelen hiçbir şeyi dikkate almaz; sadece /vehicle_command.
        """
        current_angle = self.current_steering_angle
        if self.target_steering_angle > current_angle + 2:
            self.handle_steering(1.0)
        elif self.target_steering_angle < current_angle - 2:
            self.handle_steering(-1.0)
        else:
            self.handle_steering(0.0)

        self.handle_speed(self.current_speed)

    # ====== Low-level handlers ======
    def handle_steering(self, angular_velocity_cmd):
        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1

        target_angle = self.target_steering_angle
        current_angle = self.current_steering_angle
        error = target_angle - current_angle

        # Flip/flop PWM mantığı (senkron hareket)
        if self.is_right_turned:
            steering_msg.autonomous_steeringmot_pwm = 12
            self.is_right_turned = False
            #self.get_logger().info("Direksiyon sağa döndü, şimdi nötr komumda.")
            #self.steering_publisher.publish(steering_msg)
            return

        if self.is_left_turned:
            steering_msg.autonomous_steeringmot_pwm = 140
            self.is_left_turned = False
           # self.get_logger().info("Direksiyon sola döndü, şimdi nötr konumda.")
            #self.steering_publisher.publish(steering_msg)
            return

        if angular_velocity_cmd == 0.0:
            steering_msg.autonomous_steeringmot_en = 0
            steering_msg.autonomous_steeringmot_pwm = 0
        elif angular_velocity_cmd > 0.0:
            steering_msg.autonomous_steeringmot_pwm = 140
            self.is_right_turned = True
        else:  # angular_velocity_cmd < 0
            steering_msg.autonomous_steeringmot_pwm = 12
            self.is_left_turned = True

        now = time.time()
        if now - self.last_log_time > 2.0:
            self.get_logger().info(
                f"target_angle: {target_angle:.1f}°, current_angle: {current_angle:.1f}°, "
                f"error: {error:.1f}°, pwm: {steering_msg.autonomous_steeringmot_pwm}"
            )
            self.last_log_time = now

        self.steering_publisher.publish(steering_msg)

    def handle_speed(self, linear_velocity_cmd):
        """Doğrusal hız komutunu gaz veya fren mesajına çevirir."""
        throttle_msg = Rcthrtdata()
        brake_msg = Autonomousbrakepedalcontrol()
        brake_msg.autonomous_brakepedalmotor_en = 1
        brake_msg.autonomous_brakemotor_voltage = 1
        brake_msg.autonomous_brakepedalmotor_acc = 10000

        if linear_velocity_cmd > 0.1:  # Gaza bas
            brake_msg.autonomous_brakepedalmotor_per = 0
            throttle_msg.rc_thrt_pedal_press = 0
            throttle_msg.rc_thrt_pedal_position = 60  # basit sabit (istersen hızla orantıla)
        elif linear_velocity_cmd < -0.1:  # Geri/yoğun fren
            throttle_msg.rc_thrt_pedal_press = 1
            throttle_msg.rc_thrt_pedal_position = 0
            brake_percentage = int(100 * (abs(linear_velocity_cmd) - 0.1) / 0.9)
            brake_msg.autonomous_brakepedalmotor_per = max(0, min(100, brake_percentage))
        else:  # Nötr
            throttle_msg.rc_thrt_pedal_press = 1
            throttle_msg.rc_thrt_pedal_position = 0
            brake_msg.autonomous_brakepedalmotor_per = 0

        self.brake_publisher.publish(brake_msg)
        self.throttle_publisher.publish(throttle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatılıyor.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
