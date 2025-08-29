import rclpy
from rclpy.node import Node

class YoloDepthPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_perception_node')
        self.get_logger().info('📸 YOLOv8 Derinlikli Nesne Tespiti Başlatıldı.')

def main(args=None):
    # ... (main fonksiyonu boilerplate) ...
