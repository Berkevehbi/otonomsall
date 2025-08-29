import rclpy
from rclpy.node import Node
import subprocess

class RosbagRecorderNode(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_node')
        self.get_logger().info('📦 Rosbag kayıt node başlatıldı.')

        # Kayıt yapılacak topicler
        self.topics = [
            '/zed/zed_node/left/image_rect_color',
            '/zed/zed_node/imu',
            '/zed/zed_node/gps/fix',
            '/zed/zed_node/gps/gnss',
            '/lidar/scan',
            '/beemobs/AUTONOMOUS_SteeringMot_Control',
            '/beemobs/FeedbackSteeringAngle'
        ]

        # Rosbag2 record komutu hazırlığı
        self.bag_process = None

    def start_recording(self):
        if self.bag_process is None:
            record_cmd = ['ros2', 'bag', 'record', '-o', 'my_bag'] + self.topics
            self.get_logger().info(f'🎥 Kayıt başlatılıyor: {record_cmd}')
            self.bag_process = subprocess.Popen(record_cmd)
        else:
            self.get_logger().warn('Kayıt zaten devam ediyor.')

    def stop_recording(self):
        if self.bag_process:
            self.get_logger().info('⏹️ Kayıt durduruluyor.')
            self.bag_process.terminate()  # SIGTERM gönder
            self.bag_process.wait()
            self.bag_process = None
        else:
            self.get_logger().warn('Kayıt başlamamış.')

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()

    try:
        node.start_recording()
        rclpy.spin(node)  # Node çalışmaya devam eder
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Program kesildi, kayıt durduruluyor...')
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

