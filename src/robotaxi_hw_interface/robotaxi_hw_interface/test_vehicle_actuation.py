import rclpy
from rclpy.node import Node
import time

from smart_can_msgs.msg import (
    Rcthrtdata,
    Autonomousbrakepedalcontrol,
    Autonomoussteeringmotcontrol,
    Autonomoushbmotorcontrol,
    Rcunittoomux
)

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')

        # Publishers
        self.throttle_pub = self.create_publisher(Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.brake_pub = self.create_publisher(Autonomousbrakepedalcontrol, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.steering_pub = self.create_publisher(Autonomoussteeringmotcontrol, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.handbrake_pub = self.create_publisher(Autonomoushbmotorcontrol, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.rc_unit_pub = self.create_publisher(Rcunittoomux, '/beemobs/rc_unittoOmux', 10)

        # Otomatik başlatma sekansı flag'ları
        self.systems_active = False
        self.sequence_ready = False
        self.startup_done = False

        # Test adımları için
        self.test_step = 0
        self.last_time = time.time()

        # Başlatma sekansını hemen başlat
        self.start_vehicle_sequence()

        # Test başlatıcı timer'ı (her 0.5 saniyede kontrol eder)
        self.timer = self.create_timer(0.1, self.run_test)

    def start_vehicle_sequence(self):
        """
        Aracı sürüşe hazırlayan tam otomatik başlatma sekansı.
        """
        self.get_logger().info(">>> OTOMATİK BAŞLATMA SEKANSİ BAŞLADI <<<")

        # ADIM 1: Direksiyon ve Fren Motorlarını Aktif Et
        self.get_logger().info("Adım 1: Direksiyon ve Fren motorları aktive ediliyor...")
        steering_enable_msg = Autonomoussteeringmotcontrol()
        steering_enable_msg.autonomous_steeringmot_en = 1
        self.steering_pub.publish(steering_enable_msg)
        brake_enable_msg = Autonomousbrakepedalcontrol()
        brake_enable_msg.autonomous_brakepedalmotor_en = 1
        brake_enable_msg.autonomous_brakemotor_voltage = 1
        brake_enable_msg.autonomous_brakepedalmotor_acc = 10000
        self.brake_pub.publish(brake_enable_msg)
        self.systems_active = True
        self.get_logger().info("--> Motorlar aktif.")
        time.sleep(1.0)

        # ADIM 2: Ateşleme ve Vitesi Hazırla (tek mesajda)
        self.get_logger().info("Adım 2: Ateşleme açılıyor ve vites boşa alınıyor...")
        ignition_msg = Rcunittoomux()
        ignition_msg.rc_ignition = 1
        ignition_msg.rc_selectiongear = 0  # Neutral
        self.rc_unit_pub.publish(ignition_msg)
        self.get_logger().info("--> Ateşleme AÇIK, Vites BOŞTA.")
        time.sleep(1.5)

        # ADIM 3: El Frenini İndir
        self.get_logger().info("Adım 3: El freni indiriliyor...")
        hb_msg = Autonomoushbmotorcontrol()
        hb_msg.autonomous_hb_moten = 1
        hb_msg.autonomous_hb_motstate = 1  # 1: İndir
        hb_msg.autonomous_hb_motor_pwm = 200
        self.handbrake_pub.publish(hb_msg)
        self.get_logger().info("--> El freni İNDİRİLDİ.")
        time.sleep(1.5)

        # ADIM 4: Vitesi Sürüşe Al (DRIVE)
        self.get_logger().info("Adım 4: Vites 'DRIVE' konumuna alınıyor...")
        drive_msg = Rcunittoomux()
        drive_msg.rc_ignition = 1
        drive_msg.rc_selectiongear = 1  # 1: Drive
        self.rc_unit_pub.publish(drive_msg)

        self.sequence_ready = True
        self.get_logger().info(">>> ✅ BAŞLATMA TAMAMLANDI! ARAÇ SÜRÜŞE HAZIR. <<<")

        self.startup_done = True
        self.last_time = time.time()  # Test adımlarının başlangıcı için

    def run_test(self):
        if not self.startup_done or not self.sequence_ready:
            # Başlatma sekansı tamamlanmadıysa bekle
            return

        now = time.time()
        elapsed = now - self.last_time

        # Step 0: 3 saniye boyunca gaz 60
        if self.test_step == 0:
            if elapsed < 3.0:
                self.send_throttle(60)
                self.send_brake(0)
                self.get_logger().info("Step 0: 3s Gaz (60)")
            else:
                self.test_step += 1
                self.last_time = now

        # Step 1: 2 saniye boyunca fren %50
        elif self.test_step == 1:
            if elapsed < 2.0:
                self.send_throttle(0)
                self.send_brake(50)
                self.get_logger().info("Step 1: 2s Fren (50%)")
            else:
                self.test_step += 1
                self.last_time = now

        # Step 2: 2 saniye PWM 70 + gaz 60
        elif self.test_step == 2:
            if elapsed < 2.0:
                self.send_throttle(60)
                self.send_steering_pwm(40)
                self.get_logger().info("Step 2: 2s PWM=116 + Gaz (60)")
            else:
                self.test_step += 1
                self.last_time = now

        # Step 3: 2 saniye PWM 200 + gaz 60
        elif self.test_step == 3:
            if elapsed < 2.0:
                self.send_throttle(60)
                self.send_steering_pwm(168)
                self.get_logger().info("Step 3: 2s PWM=140 + Gaz (60)")
            else:
                self.test_step += 1
                self.last_time = now

        # Step 4: Araç dursun (her şeyi bırak)
        elif self.test_step == 4:
            self.send_throttle(0)
            self.send_brake(0)
            self.get_logger().info("Step 4: Araç durdu. Test bitti.")
            self.test_step += 1  # Bir sonraki çağrıda hiçbir şey yapma (test tamamlandı)

    def send_throttle(self, position):
        msg = Rcthrtdata()
        msg.rc_thrt_pedal_press = 1 if position == 0 else 0
        msg.rc_thrt_pedal_position = position
        self.throttle_pub.publish(msg)

    def send_steering_pwm(self, pwm):
        msg = Autonomoussteeringmotcontrol()
        msg.autonomous_steeringmot_en = 1 if pwm > 0 else 0
        msg.autonomous_steeringmot_pwm = pwm
        self.steering_pub.publish(msg)

    def send_brake(self, percent):
        msg = Autonomousbrakepedalcontrol()
        msg.autonomous_brakepedalmotor_en = 1 if percent > 0 else 0
        msg.autonomous_brakepedalmotor_per = percent
        self.brake_pub.publish(msg)

    def send_brake_pwm(self, pwm):
        msg = Autonomousbrakepedalcontrol()
        msg.autonomous_brakepedalmotor_en = 1 if pwm > 0 else 0
        msg.autonomous_brakepedalmotor_per = pwm
        self.brake_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
