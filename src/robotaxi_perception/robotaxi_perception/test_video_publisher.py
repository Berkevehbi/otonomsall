# test_video_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class TestVideoPublisher(Node):
    def __init__(self):
        super().__init__('test_video_publisher')

        # Parametreleri tanımla
        self.declare_parameter('video_path', '/full_ws/src/robotaxi_perception/robotaxi_perception/zed_video_output.mp4')
        self.declare_parameter('camera_topic', '/zed/zed_node/rgb/image_raw_color')
        self.declare_parameter('fps', 30.0)

        # Parametreleri al
        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        # Publisher ve CvBridge'i oluştur
        self.publisher_ = self.create_publisher(Image, camera_topic, 10)
        self.bridge = CvBridge()

        # Videoyu aç
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Video dosyası açılamadı: {video_path}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Video yayınlanıyor: {video_path} -> {camera_topic}")

        # Belirtilen FPS'e göre timer oluştur
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Frame'i ROS Image mesajına çevir ve yayınla
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().info('Video sona erdi, başa sarılıyor.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    node = TestVideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()