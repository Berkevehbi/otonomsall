#!/usr/bin/env python3

# ros2 topic echo /vehicle_command
# ros2 topic echo /beemobs/RC_THRT_DATA

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import threading
import time

# Komut ayarları
MAX_SPEED = 1.0  # Maksimum ileri/geri hız (-1.0 ile 1.0 arası)
MAX_TURN = 1.0   # Maksimum dönüş hızı (-1.0 ile 1.0 arası)
SPEED_STEP = 0.1 # Her tuşa basışta hızın ne kadar artacağı/azalacağı
TURN_STEP = 0.1  # Her tuşa basışta dönüşün ne kadar artacağı/azalacağı

# Kullanıcıya gösterilecek talimatlar
instructions = """
Kontrol Tuşları:
---------------------------
    w/s : Hızı arttır / azalt
    a/d : Sola / Sağa dönüşü arttır/azalt

    q   : Hız ve dönüşü yavaşça sıfırla
    SPACE : ACİL DURUŞ (Hız ve dönüşü anında sıfırla)

CTRL-C ile çıkış yapabilirsiniz.
---------------------------
"""

# Terminalden anlık tuş okumak için yardımcı fonksiyon
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.get_logger().info("Klavye Teleop Düğümü Başlatıldı. Komutlar için diğer terminali kullanın.")

        # Sadece TEK BİR publisher'a ihtiyacımız var!
        self.publisher_ = self.create_publisher(Twist, '/vehicle_command', 10)

        # Anlık hız ve dönüş değerlerini saklayan değişkenler
        self.speed = 0.0
        self.turn = 0.0

        # Terminal ayarlarını sakla
        self.settings = termios.tcgetattr(sys.stdin)

        # Ayrı bir thread'de klavye dinleme ve komut gönderme döngüsü
        self.monitor_thread = threading.Thread(target=self.key_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def key_loop(self):
        """Klavye girdilerini dinler, hız/dönüş değerlerini günceller ve yayınlar."""
        print(instructions)
        while rclpy.ok():
            key = get_key(self.settings)

            if key != '':
                # Hız kontrolü (w/s)
                if key == 'w':
                    self.speed = min(MAX_SPEED, self.speed + SPEED_STEP)
                elif key == 's':
                    self.speed = max(-MAX_SPEED, self.speed - SPEED_STEP)
                # Dönüş kontrolü (a/d)
                elif key == 'a':
                    self.turn = min(MAX_TURN, self.turn + TURN_STEP)
                elif key == 'd':
                    self.turn = max(-MAX_TURN, self.turn - TURN_STEP)
                # Yavaşça sıfırlama
                elif key == 'q':
                    self.speed *= 0.9
                    self.turn *= 0.9
                # Acil duruş (space)
                elif key == ' ':
                    self.speed = 0.0
                    self.turn = 0.0
                # CTRL-C
                elif (key == '\x03'):
                    break
            else:
                # Tuşa basılmıyorsa hızı ve dönüşü yavaşça azalt (isteğe bağlı)
                # Bu özellik aracın daha yumuşak durmasını sağlar.
                self.speed *= 0.98
                self.turn *= 0.90

            # Her döngüde güncel durumu terminale yazdır
            print(f"Hız: {self.speed:+.2f} | Dönüş: {self.turn:+.2f}", end='\r')

            # Twist mesajını oluştur ve yayınla
            twist_msg = Twist()
            twist_msg.linear.x = self.speed
            twist_msg.angular.z = self.turn
            self.publisher_.publish(twist_msg)

            # Döngünün çok hızlı çalışmasını engellemek için küçük bir bekleme
            time.sleep(0.05) # 20 Hz

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboardNode()
    
    # Düğümün kapanmaması için spin atıyoruz, asıl iş thread'de dönüyor.
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Program kapanırken terminal ayarlarını eski haline getir
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()