# teleop_keyboard_node.py (Düzeltilmiş ve Geliştirilmiş Hız Kontrolü)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

# Kontrol ayarları
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
LINEAR_STEP_SIZE = 0.05  # Daha hassas kontrol için adımı küçülttük
ANGULAR_STEP_SIZE = 0.1

# Tuş haritası
key_bindings = {
    'w': (LINEAR_STEP_SIZE, 0.0),
    's': (-LINEAR_STEP_SIZE, 0.0),
    'a': (0.0, ANGULAR_STEP_SIZE),
    'd': (0.0, -ANGULAR_STEP_SIZE),
    'x': (-LINEAR_STEP_SIZE, 0.0),
}

help_text = """
Robotaxi'yi klavye ile kontrol etme
---------------------------
i : Aracı Sürüşe Hazırla (Ignition ON)
w/a/s/d : Hareket (Basılı tutarak hızlan)
space key, k : Anında Dur
CTRL-C ile çıkış yapabilirsiniz.
"""

def getKey(settings):
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
        self.twist_publisher = self.create_publisher(Twist, '/vehicle_command', 10)
        self.system_publisher = self.create_publisher(String, '/system_command', 10)
        
        self.speed = 0.0
        self.turn = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Teleop Keyboard Düğümü Başlatıldı.")
        print(help_text)
        
        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        key = getKey(self.settings)

        # 'i' tuşu başlatma sekansını tetikler
        if key == 'i':
            start_msg = String()
            start_msg.data = "start_sequence"
            self.system_publisher.publish(start_msg)
            print("BAŞLATMA SEKANSI TALEBİ GÖNDERİLDİ!")
        
        # Hareket kontrolü
        if key in key_bindings.keys():
            # DÜZELTME: Hızı ayarlamak için "=" yerine "+=" kullanıyoruz
            self.speed += key_bindings[key][0]
            self.turn += key_bindings[key][1]
        elif key == ' ' or key == 'k':
            self.speed = 0.0
            self.turn = 0.0
        else:
            # Tuşa basılmıyorsa hız ve dönüşü yavaşça azalt (daha akıcı bir duruş için)
            self.speed *= 0.90
            self.turn *= 0.90
        
        # Hız ve dönüşü limitler içinde tut
        self.speed = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, self.speed))
        self.turn = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, self.turn))

        # Kullanıcıya durumu bildir
        print(f"Hız: {self.speed:+.2f} m/s | Dönüş: {self.turn:+.2f} rad/s", end='\r')

        # Her döngüde güncel komutu yayınla
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.turn
        self.twist_publisher.publish(twist)

    def restore_terminal_settings(self):
         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.restore_terminal_settings()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()