# ros2 topic pub --rate 10 /vehicle_command geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
# ros2 topic pub --rate 10 /vehicle_command geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: -0.5}}"
# ros2 topic pub /vehicle_command geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from smart_can_msgs.msg import (
    Rcunittoomux,
    Rcthrtdata,
    Autonomousbrakepedalcontrol,
    Autonomoussteeringmotcontrol,
    Autonomoushbmotorcontrol
)
from std_msgs.msg import String, Float64, Bool
from smart_can_msgs.msg import Feedbacksteeringangle as FeedbackSteeringAngle

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hw_interface_node')
        self.get_logger().info('🚗 Geliştirilmiş Donanım Arayüz Düğümü Başlatıldı.')

        # =============== STATE VARIABLES ===============
        self.current_speed = 0.0      # En son istenen doğrusal hız
        self.current_steer = 0.0      # En son istenen açısal hız
        self.systems_active = False   # Kontrol sistemleri (fren/direksiyon motorları) aktif mi?
        self.sequence_ready = False   # Araç sürüşe hazır mı? (Ateşleme, vites, el freni OK)
        self.feedback_yon= False  # Sollama ya da sağa dönme durumu
        self.current_steering_angle = 0.0  # En son direksiyon aç
        self.target_direction = None  # Hedef yön (left/right/straight)
        self.stopIsIt=False
        self.stopFollower=False
        self.previous_direction=''

        self.previous_stop = False  # Önceki stop durumu
        self.stopIsIt = False  # Şu anki stop durumu
        

        # =============== SUBSCRIBERS ===============
        # Sadece harici sürüş komutlarını dinliyoruz
        self.command_subscriber = self.create_subscription(
            Twist, '/vehicle_command', self.vehicle_command_callback, 10)

        # =============== PUBLISHERS ===============
        self.rc_unit_publisher = self.create_publisher(Rcunittoomux, '/beemobs/rc_unittoOmux', 10)
        self.throttle_publisher = self.create_publisher(Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.steering_publisher = self.create_publisher(Autonomoussteeringmotcontrol, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.brake_publisher = self.create_publisher(Autonomousbrakepedalcontrol, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.handbrake_publisher = self.create_publisher(Autonomoushbmotorcontrol, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        # =============== SUBSCRIBERS ===============
        self.create_subscription(
            Bool,
            '/feedback_yon',
            self.feedback_yon_callback,
            10
        )
        self.create_subscription(
            String,
            '/target_direction',
            self.target_direction_callback,
            10
        )
        self.create_subscription(
            FeedbackSteeringAngle,
            '/beemobs/FeedbackSteeringAngle',
            self.steering_angle_callback,
            10
        )
        self.subscription = self.create_subscription(
        Bool,
        '/is_stop',
        self.is_stop_callback,
        10)


        # =============== TIMERS ===============
        # 1. Periyodik Kontrol (50Hz): Sürekli olarak komutları donanıma gönderir.
        self.periodic_control_timer = self.create_timer(0.02, self.periodic_publish)

        # 2. Otomatik Başlatma Sekansı: Düğüm başladıktan 2 saniye sonra SADECE BİR KEZ çalışır.
        self.startup_timer = self.create_timer(2.0, self.start_vehicle_sequence)

        self.get_logger().info("Düğüm hazır. 2 saniye içinde otomatik başlatma sekansı başlayacak...")

    def start_vehicle_sequence(self):
        """
        Aracı sürüşe hazırlayan tam otomatik başlatma sekansı.
        Bu fonksiyon sadece bir kez çalışır ve sonra kendini imha eder.
        Sıralama ve beklemeler, eski çalışan kodlardan alınan bilgilere göredir.
        """
        self.get_logger().info(">>> OTOMATİK BAŞLATMA SEKANSİ BAŞLADI <<<")

        # ADIM 1: Direksiyon ve Fren Motorlarını Aktif Et
        self.get_logger().info("Adım 1: Direksiyon ve Fren motorları aktive ediliyor...")
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
        time.sleep(1.0) # Motorların kendine gelmesi için bekle

        # ADIM 2: Ateşleme ve Vitesi Hazırla (tek mesajda)
        self.get_logger().info("Adım 2: Ateşleme açılıyor ve vites boşa alınıyor...")
        ignition_msg = Rcunittoomux()
        ignition_msg.rc_ignition = 1
        ignition_msg.rc_selectiongear = 0 # Önce N (Neutral - Boş Vites)
        self.rc_unit_publisher.publish(ignition_msg)
        self.get_logger().info("--> Ateşleme AÇIK, Vites BOŞTA.")
        time.sleep(1.5) # ECU'nun ateşlemeyi algılaması için bekle

        # ADIM 3: El Frenini İndir
        self.get_logger().info("Adım 3: El freni indiriliyor...")
        hb_msg = Autonomoushbmotorcontrol()
        hb_msg.autonomous_hb_moten = 1
        hb_msg.autonomous_hb_motstate = 1 # 1: İndir
        hb_msg.autonomous_hb_motor_pwm = 200
        self.handbrake_publisher.publish(hb_msg)
        self.get_logger().info("--> El freni İNDİRİLDİ.")
        time.sleep(1.5) # El freni motorunun işlemi bitirmesi için bekle

        # ADIM 4: Vitesi Sürüşe Al (DRIVE)
        self.get_logger().info("Adım 4: Vites 'DRIVE' konumuna alınıyor...")
        drive_msg = Rcunittoomux()
        drive_msg.rc_ignition = 1 # Ateşleme bilgisini tekrar göndermek genellikle güvenlidir
        drive_msg.rc_selectiongear = 1 # 1: Drive
        self.rc_unit_publisher.publish(drive_msg)
        
        # Sekans tamamlandı, araç sürüş komutlarını dinlemeye hazır.
        self.sequence_ready = True
        self.get_logger().info(">>> ✅ BAŞLATMA TAMAMLANDI! ARAÇ SÜRÜŞE HAZIR. <<<")
        
        # Bu timer'a bir daha ihtiyacımız yok, iptal edelim.
        self.startup_timer.cancel()

    def vehicle_command_callback(self, msg: Twist):
        """ Gelen anlık hız ve direksiyon komutunu değişkenlere kaydeder. """
        self.current_speed = msg.linear.x
        self.current_steer = msg.angular.z

    def periodic_publish(self):
        """
        50Hz'de sürekli çalışır. Araç hazırsa, en son komutları donanıma gönderir.
        """
        # Güvenlik Kontrolü: Başlatma sekansı bitmeden asla komut gönderme!
        if not self.systems_active or not self.sequence_ready:
            # Henüz hazır değiliz, bu periyodu atla.
            # İsteğe bağlı olarak burada bir uyarı logu basılabilir ama 50Hz'de terminali doldurur.
            return
        
        if self.feedback_yon:
            if not self.stopIsIt:
                if self.target_direction == "left":
                    self.left()
                elif self.target_direction == "right":
                    self.right()
            else:
                if self.target_direction == "left":
                    self.left()
                elif self.target_direction == "right":
                    self.right()
        else:
            # feedback_yon == False durumunda
            # Önceki stop True, şu anki stop False ise yönü tersine çevir
            if self.previous_stop and not self.stopIsIt:
                if self.previous_direction == "left":
                    self.right()
                elif self.previous_direction == "right":
                    self.left()
        # En sonda mevcut stop durumunu bir sonraki tur için sakla
        self.previous_stop = self.stopIsIt
        self.previous_direction = self.target_direction

    def handle_steering(self, angular_velocity_cmd):
        """Açısal hız komutunu direksiyon PWM değerine çevirir."""
        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1 # Her mesajda aktif olduğunu belirtmek önemlidir

        # -1.0 ile +1.0 arasındaki komutu 0-255 PWM aralığına haritala
        if angular_velocity_cmd < -0.1: # Sola dönüş
            pwm_value = int(127 * (1 - (abs(angular_velocity_cmd) - 0.1) / 0.9))
            steering_msg.autonomous_steeringmot_pwm = max(0, min(127, pwm_value))
        elif angular_velocity_cmd > 0.1: # Sağa dönüş
            pwm_value = int(128 + 127 * ((angular_velocity_cmd - 0.1) / 0.9))
            steering_msg.autonomous_steeringmot_pwm = max(128, min(255, pwm_value))
        else: # Nötr bölge
            steering_msg.autonomous_steeringmot_pwm = 128

        self.steering_publisher.publish(steering_msg)
    

    def handle_speed(self, linear_velocity_cmd):
        """Doğrusal hız komutunu gaz veya fren mesajına çevirir."""
        throttle_msg = Rcthrtdata()
        brake_msg = Autonomousbrakepedalcontrol()
        # Fren motorunun sürekli aktif kalması için bu değerleri her döngüde gönderiyoruz
        brake_msg.autonomous_brakepedalmotor_en = 1
        brake_msg.autonomous_brakemotor_voltage = 1
        brake_msg.autonomous_brakepedalmotor_acc = 10000

        if linear_velocity_cmd > 0.1: # Gaza bas
            brake_msg.autonomous_brakepedalmotor_per = 0
            throttle_position = int(60 + 200 * ((linear_velocity_cmd - 0.1) / 0.9))
            throttle_msg.rc_thrt_pedal_press = 0
            throttle_msg.rc_thrt_pedal_position = max(60, min(250, throttle_position))
        elif linear_velocity_cmd < -0.1: # Frene bas
            throttle_msg.rc_thrt_pedal_press = 1
            throttle_msg.rc_thrt_pedal_position = 0
            brake_percentage = int(100 * (abs(linear_velocity_cmd) - 0.1) / 0.9)
            brake_msg.autonomous_brakepedalmotor_per = max(0, min(100, brake_percentage))
        else: # Nötr bölge (gaz ve freni bırak)
            throttle_msg.rc_thrt_pedal_press = 1
            throttle_msg.rc_thrt_pedal_position = 0
            brake_msg.autonomous_brakepedalmotor_per = 0

        throttle_msg.rc_thrt_pedal_position = 60
        self.brake_publisher.publish(brake_msg)
        self.throttle_publisher.publish(throttle_msg)
    def send_throttle(self, position):
        msg = Rcthrtdata()
        msg.rc_thrt_pedal_position = 60
        msg.rc_thrt_pedal_press = 0 if position > 0 else 1
        self.thrt_pub.publish(msg)

    def send_brake(self, enable, pwm=0):
        msg = Autonomousbrakepedalcontrol()
        msg.autonomous_brakemotor_voltage = 1
        msg.autonomous_brakepedalmotor_per = pwm if enable else 0
        msg.autonomous_brakepedalmotor_acc = 65000 if enable else 0
        msg.autonomous_brakepedalmotor_en = 1 if enable else 0
        self.brake_pub.publish(msg)

    def release_el_freni(self):
        msg = Autonomoushbmotorcontrol()
        msg.autonomous_hb_motor_pwm = 200
        msg.autonomous_hb_motstate = 1
        msg.autonomous_hb_moten = 1
        self.hb_pub.publish(msg)

    def apply_el_freni(self):
        msg = Autonomoushbmotorcontrol()
        msg.autonomous_hb_motor_pwm = 200
        msg.autonomous_hb_motstate = 0
        msg.autonomous_hb_moten = 1
        self.hb_pub.publish(msg)

    def send_ignition_and_gear(self):
        msg = Rcunittoomux()
        msg.rc_ignition = 1
        msg.rc_selectiongear = 1
        msg.autonomous_emergency = 0
        self.unittomux_pub.publish(msg)
        self.get_logger().info("🟢 Ateşleme verildi ve vites ileri alındı.")

    def shutdown_procedure(self):
        self.get_logger().warn("🔴 Node kapatılıyor. Gaz kes, fren uygula, el freni çek.")
        self.send_throttle(0)
        self.send_brake(True, 100)
        self.apply_el_freni()
    def stop_vehicle(self):
        self.get_logger().info("Araç durduruluyor...")
        self.send_throttle(0)
        self.send_brake(True, 100)
        self.apply_el_freni()
        self.get_logger().info("Araç durduruldu.")
    def steering_angle_callback(self, msg: FeedbackSteeringAngle):
        self.current_steering_angle = msg.feedbacksteeringangle
        self.get_logger().info(f"Current steering angle: {self.current_steering_angle:.2f}°")
    def target_direction_callback(self, msg: String):
        self.target_direction = msg.data.lower()  # örnek: "left", "right", "straight"
        self.get_logger().info(f"Target direction: {self.target_direction}")
    def feedback_yon_callback(self, msg: Bool):
        self.feedback_yon = msg.data
        self.get_logger().info(f"Feedback direction: {self.feedback_yon}")
    def straight(self):
        self.get_logger().info("Araç düz yolda ilerliyor...")
        self.send_throttle(60)
    def left(self):
        self.get_logger().info("Araç sola dönüyor...")
        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1 # Her mesajda aktif olduğunu belirtmek önemlidir
        steering_msg.autonomous_steeringmot_pwm=70
        self.send_throttle(50)
    def right(self):
        self.get_logger().info("Araç sola dönüyor...")
        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1 # Her mesajda aktif olduğunu belirtmek önemlidir
        steering_msg.autonomous_steeringmot_pwm=200
        self.send_throttle(50)
    def is_stop_callback(self, msg):
        self.stopIsIt = msg.data
    def right_stop(self):
        self.get_logger().info("3 saniye boyunca sağa dönülüyor...")
        # Sağa dönmeyi başlat
        self.right()
        # 3 saniye sonra stop_steering() fonksiyonunu çağır
        self.timer = self.create_timer(3.0, self.stop_steering_once)
        self.previous_stop=False
    def stop_steering_once(self):
        self.get_logger().info("Sağa dönüş durduruluyor.")
        self.stop_steering()
        # Timer sadece bir kere çalışsın diye kapat
        self.timer.cancel()
    def stop_steering(self):
        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 0
        steering_msg.autonomous_steeringmot_pwm = 128
        self.send_throttle(0)
        self.get_logger().info("Direksiyon durduruldu.")
        self.get_logger().info("10 saniye bekleniyor...")
        time.sleep(10)
    def left_stop(self):
        self.get_logger().info("3 saniye boyunca sola dönülüyor..."),
        # Sola dönmeyi başlat
        self.left()
        # 3 saniye sonra stop_steering() fonksiyonunu çağır
        self.left_timer = self.create_timer(3.0, self.stop_steering_left_once)
        self.previous_stop=False
    def stop_steering_left_once(self):
        self.get_logger().info("Sola dönüş durduruluyor.")
        self.stop_steering()

        # Timer'ı kapat
        self.left_timer.cancel()




        

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