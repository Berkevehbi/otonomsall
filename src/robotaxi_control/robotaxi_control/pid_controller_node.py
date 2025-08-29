import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time
import numpy as np

# Hız mesajını import ediyoruz
from smart_can_msgs.msg import Fbvehiclespeed

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.get_logger().info('Hıza Duyarlı PID Kontrolcü Düğümü Başlatıldı.')

        # === Hıza Duyarlı PID Parametreleri (km/h'e göre güncellendi) ===
        self.declare_parameter('kp_low_speed', 0.3)
        self.declare_parameter('kd_low_speed', 0.05)
        self.declare_parameter('kp_high_speed', 0.1)
        self.declare_parameter('kd_high_speed', 0.2)
        # GÜNCELLEME: Eşik değerleri ve yorumları km/h cinsinden
        self.declare_parameter('low_speed_threshold', 2.0)  # km/h
        self.declare_parameter('high_speed_threshold', 10.0) # km/h
        
        # Diğer parametreler
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kf', 0.0) # Feedforward
        self.declare_parameter('target_speed', 0.3)
        self.declare_parameter('max_steering_angle', 1.0)

        # === ROS2 Altyapısı ===
        self.offset_subscriber = self.create_subscription(
            Float32, '/planning/lane_offset_m', self.offset_callback, 10)
        
        self.curvature_subscriber = self.create_subscription(
            Float32, '/planning/lane_curvature_radius_m', self.curvature_callback, 10)
        
        self.speed_subscriber = self.create_subscription(
            Fbvehiclespeed,
            '/beemobs/FB_VehicleSpeed',
            self.speed_callback,
            10)
            
        #self.command_publisher = self.create_publisher(Twist, '/vehicle_command', 10)
        self.steering_pub = self.create_publisher(Twist, '/pid/steering_cmd', 10)

        # === Durum Değişkenleri ===
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        # GÜNCELLEME: Değişken adını kmh olarak değiştirdik
        self.current_speed_kmh = 0.0
        self.current_curvature = float('inf')

        self.previous_steering_command = 0.0  # __init__ içinde, başlangıçta ekle
        
        self.get_logger().info('PID Kontrolcü hazır. Planlayıcıdan offset, eğrilik ve hız bekleniyor...')

    def limit_rate(self, new_value, old_value, max_delta):
        delta = new_value - old_value
        if delta > max_delta:
            return old_value + max_delta
        elif delta < -max_delta:
            return old_value - max_delta
        else:
            return new_value


    def speed_callback(self, msg: Fbvehiclespeed):
        """ Gelen anlık hız verisini kaydeder. """
        # GÜNCELLEME: Artık m/s yerine km/h okuyoruz
        self.current_speed_kmh = float(msg.fb_reelvehiclespeed_kmh)

    def curvature_callback(self, msg: Float32):
        """ Gelen anlık eğrilik yarıçapı verisini kaydeder. """
        self.current_curvature = msg.data

    def offset_callback(self, msg: Float32):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return
        self.last_time = current_time

        error = msg.data

        # === Hıza Duyarlı Gain Scheduling ===
        kp_low = self.get_parameter('kp_low_speed').get_parameter_value().double_value
        kd_low = self.get_parameter('kd_low_speed').get_parameter_value().double_value
        kp_high = self.get_parameter('kp_high_speed').get_parameter_value().double_value
        kd_high = self.get_parameter('kd_high_speed').get_parameter_value().double_value
        low_speed = self.get_parameter('low_speed_threshold').get_parameter_value().double_value
        high_speed = self.get_parameter('high_speed_threshold').get_parameter_value().double_value

        # Anlık hıza göre Kp ve Kd'yi enterpolasyon ile hesapla
        current_kp = np.interp(self.current_speed_kmh, [low_speed, high_speed], [kp_low, kp_high])
        current_kd = np.interp(self.current_speed_kmh, [low_speed, high_speed], [kd_low, kd_high])

        # PID terimleri
        p_term = current_kp * error
        self.integral += error * dt
        i_term = self.get_parameter('ki').get_parameter_value().double_value * self.integral
        derivative = (error - self.previous_error) / dt
        d_term = current_kd * derivative
        self.previous_error = error

        # Feedforward Terimi
        kf = self.get_parameter('kf').get_parameter_value().double_value
        ff_term = 0.0
        if self.current_curvature is not None and np.isfinite(self.current_curvature) and abs(self.current_curvature) > 0.1:
            ff_term = kf / self.current_curvature

        steering_command = p_term + i_term + d_term - ff_term

        #self.get_logger().info(
        #    f'PID Komutu: P={p_term:.3f}, I={i_term:.3f}, D={d_term:.3f}, FF={ff_term:.3f}'
        #)

        max_steer = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        steering_command = -steering_command
        steering_command = np.clip(steering_command, -max_steer, max_steer)

        # === RATE LIMITING ===
        max_delta = 0.1  # Buradaki değeri ihtiyaca göre ayarla (ör: 0.05, 0.1, vs)
        steering_command = self.limit_rate(
            steering_command,
            getattr(self, "previous_steering_command", 0.0),  # Varsayılanı sıfır, ilk seferde sorun çıkarmaz
            max_delta
        )
        self.previous_steering_command = steering_command  # PREVIOUS COMMAND

        #self.get_logger().info(
        #    f'Hız:{self.current_speed_kmh:.2f}km/h | Kp:{current_kp:.2f} | Kd: {current_kd:.2f} Hata:{error:.3f}m | Komut:{steering_command:.3f}'
        #)

        twist_msg = Twist()
        twist_msg.linear.x = self.get_parameter('target_speed').get_parameter_value().double_value
        twist_msg.angular.z = steering_command
        #self.command_publisher.publish(twist_msg)
        self.steering_pub.publish(twist_msg)

# main fonksiyonu aynı kalıyor...
def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()