import rclpy
from rclpy.node import Node

class UmsInterfaceNode(Node):
    def __init__(self):
        super().__init__('ums_interface_node')
        self.get_logger().info('📡 UMS Arayüzü (Go/E-Stop) Başlatıldı.')

def main(args=None):
    # ... (main fonksiyonu boilerplate) ...
