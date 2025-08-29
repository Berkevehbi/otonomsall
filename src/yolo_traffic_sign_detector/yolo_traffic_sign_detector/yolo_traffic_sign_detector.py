import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class YoloTrafficSignDetector(Node):
    def __init__(self):
        super().__init__('yolo_traffic_sign_detector')
        # --- Configurable params ---
        self.input_topic = '/zed/zed_node/left/image_rect_color'
        self.output_topic = '/yolo_traffic_sign/detected_image'
        # --- CV bridge ---
        self.bridge = CvBridge()

        # --- Paths (package içinden dinamik) ---
        package_share = get_package_share_directory('yolo_traffic_sign_detector')
        model_path = str(Path(package_share) / 'model' / 'my_model.onnx')
        data_yaml_path = str(Path(package_share) / 'data' / 'data.yaml')

        # --- Load YOLO model ---
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded from: {model_path}")

        # --- Load class names from YAML ---
        with open(data_yaml_path, 'r', encoding='utf-8') as f:
            self.class_names = yaml.safe_load(f)['names']
        self.get_logger().info(f"Loaded {len(self.class_names)} traffic sign classes from YAML.")

        # --- ROS subscriptions & publishers ---
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(
            Image, self.output_topic, 10)
        self.get_logger().info(f"Subscribed to: {self.input_topic}")
        self.get_logger().info(f"Publishing annotated images to: {self.output_topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)[0]  # YOLOv8 output

            # Optional: Print detected objects
            if hasattr(results, 'boxes') and results.boxes is not None:
                for box in results.boxes:
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
                    self.get_logger().info(f"Detected: {label} (conf: {conf:.2f})")
            else:
                self.get_logger().info("No detections.")

            # Draw boxes (YOLO .plot() returns annotated image)
            annotated = results.plot()
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.publisher.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrafficSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
