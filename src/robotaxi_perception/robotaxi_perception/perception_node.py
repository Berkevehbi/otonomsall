# perception_node.py (Nihai Import Düzeltmesi)
import sys
import os

# --- SENİN ÇALIŞAN KODUNDAN ALINAN ÇÖZÜM ---
# Bu kod, 'lib' klasörünü içeren ana klasörü Python'un arama yoluna ekler.
# Bu sayede 'lib' içindeki dosyalar birbirlerini 'from lib.utils...' gibi bulabilir.
# Bu dosyanın bulunduğu klasör ('.../robotaxi_perception/robotaxi_perception/')
package_source_path = os.path.dirname(os.path.abspath(__file__))
# Arama yoluna ekle
sys.path.insert(0, package_source_path)
# --- ÇÖZÜM SONU ---

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

from ament_index_python.packages import get_package_share_directory

# Artık 'lib' path'e eklendiği için importlar bu şekilde çalışacaktır
from lib.config import cfg
from lib.config import update_config
from lib.models import get_net
from lib.utils.utils import select_device
from lib.core.postprocess import morphological_process
import torchvision.transforms as transforms

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('Algılama Düğümü (Perception Node) Başlatılıyor...')

        # === Genel Parametreler ===
        self.declare_parameter('weights_path', 'weights/End-to-end.pth')
        self.declare_parameter('config_path', 'lib/config/bdd100k.yaml')
        self.declare_parameter('img_size', 640)
        self.declare_parameter('camera_topic', '/zed/zed_node/rgb_raw/image_raw_color')

        # YENİ: Akıllı ROI Yamuğunun Köşeleri İçin Canlı Ayarlanabilir Parametreler
        # Bu varsayılan değerler, tipik bir 1280x720 görüntü için iyi bir başlangıçtır.
        """
        self.declare_parameter('roi_p1_x', 200)   # Sol Alt X
        self.declare_parameter('roi_p1_y', 720)   # Sol Alt Y
        self.declare_parameter('roi_p2_x', 1280)  # Sağ Alt X
        self.declare_parameter('roi_p2_y', 720)   # Sağ Alt Y
        self.declare_parameter('roi_p3_x', 930)   # Sağ Üst X
        self.declare_parameter('roi_p3_y', 330)   # Sağ Üst Y
        self.declare_parameter('roi_p4_x', 530)   # Sol Üst X
        self.declare_parameter('roi_p4_y', 330)   # Sol Üst Y
        """
        self.declare_parameter('roi_p1_x', 0)     # Sol Alt X
        self.declare_parameter('roi_p1_y', 720)   # Sol Alt Y
        self.declare_parameter('roi_p2_x', 1280)  # Sağ Alt X
        self.declare_parameter('roi_p2_y', 720)   # Sağ Alt Y
        self.declare_parameter('roi_p3_x', 1280)  # Sağ Üst X
        self.declare_parameter('roi_p3_y', 0)     # Sağ Üst Y
        self.declare_parameter('roi_p4_x', 0)     # Sol Üst X
        self.declare_parameter('roi_p4_y', 0)     # Sol Üst Y
        
        # === Kurulum ===
        package_share_directory = get_package_share_directory('robotaxi_perception')
        weights_path = os.path.join(package_share_directory, self.get_parameter('weights_path').get_parameter_value().string_value)
        config_path = os.path.join(package_share_directory, self.get_parameter('config_path').get_parameter_value().string_value)
        self.img_size = self.get_parameter('img_size').get_parameter_value().integer_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"======= MODEL CİHAZI BAŞARIYLA AYARLANDI: {self.device} =======")
        self.model = self.load_model(weights_path, config_path)
        
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.lane_mask_pub = self.create_publisher(Image, '/perception/lane_mask', 10)
        self.drivable_mask_pub = self.create_publisher(Image, '/perception/drivable_area_mask', 10)
        self.debug_image_pub = self.create_publisher(Image, '/perception/debug_image', 10)
        
        self.get_logger().info('Algılama Düğümü sürüşe hazır. Kamera bekleniyor...')

    def load_model(self, weights_path, config_path):
        class Args:
            cfg = config_path
            modelDir, logDir, dataDir, prevModelDir = '', '', '', ''
        args = Args()
        update_config(cfg, args)
        model = get_net(cfg)
        try:
            checkpoint = torch.load(weights_path, map_location=self.device, weights_only=True)
        except:
            self.get_logger().warn("`weights_only=True` ile yükleme başarısız. `weights_only=False` ile deneniyor.")
            checkpoint = torch.load(weights_path, map_location=self.device, weights_only=False)
        model.load_state_dict(checkpoint['state_dict'])
        model.to(self.device)
        model.eval()
        return model

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Hatası: {e}')
            return
            
        # Model ile inference yap ve maskeleri al
        img_det = cv2.resize(cv_image, (self.img_size, self.img_size))
        img_tensor = self.transform(img_det).to(self.device)
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)
        with torch.no_grad():
            det_out, da_seg_out, ll_seg_out = self.model(img_tensor)

        _, da_seg_mask = torch.max(da_seg_out, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy().astype(np.uint8)
        da_seg_mask = cv2.resize(da_seg_mask, (cv_image.shape[1], cv_image.shape[0]))

        _, ll_seg_mask = torch.max(ll_seg_out, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy().astype(np.uint8)
        ll_seg_mask = cv2.resize(ll_seg_mask, (cv_image.shape[1], cv_image.shape[0]))
        
        # YENİ: Akıllı ROI Maskesini Oluştur ve Uygula
        roi_mask = np.zeros_like(ll_seg_mask)
        p1 = (self.get_parameter('roi_p1_x').get_parameter_value().integer_value, self.get_parameter('roi_p1_y').get_parameter_value().integer_value)
        p2 = (self.get_parameter('roi_p2_x').get_parameter_value().integer_value, self.get_parameter('roi_p2_y').get_parameter_value().integer_value)
        p3 = (self.get_parameter('roi_p3_x').get_parameter_value().integer_value, self.get_parameter('roi_p3_y').get_parameter_value().integer_value)
        p4 = (self.get_parameter('roi_p4_x').get_parameter_value().integer_value, self.get_parameter('roi_p4_y').get_parameter_value().integer_value)
        roi_vertices = np.array([[p1, p2, p3, p4]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        
        # Orijinal şerit maskesini bu yeni ROI ile kırp
        ll_seg_mask_roi = cv2.bitwise_and(ll_seg_mask, ll_seg_mask, mask=roi_mask)

        # Morfolojik işlemleri ROI uygulanmış maskeye yap
        mask255 = (ll_seg_mask_roi * 255).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        cleaned = cv2.morphologyEx(mask255, cv2.MORPH_OPEN, kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=1)
        final_lane_mask = (cleaned > 127).astype(np.uint8)

        try:
            # path_planner'a artık sadece ROI içindeki temizlenmiş şeritleri gönder
            self.lane_mask_pub.publish(self.bridge.cv2_to_imgmsg((final_lane_mask * 255).astype(np.uint8), "mono8"))
            self.drivable_mask_pub.publish(self.bridge.cv2_to_imgmsg(da_seg_mask, "mono8"))
            # Debug görüntüsünü ROI çerçevesiyle birlikte oluştur
            debug_image = self.visualize_results(cv_image, final_lane_mask, da_seg_mask, roi_vertices)
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Yayınlama Hatası: {e}')

    def visualize_results(self, img, lane_mask, drivable_mask, roi_vertices):
        vis_img = img.copy()

        # Sürüş yapılabilir alanı ve şerit maskesini çiz
        drivable_overlay = np.zeros_like(vis_img, dtype=np.uint8)
        drivable_overlay[drivable_mask == 1] = (0, 255, 0)
        vis_img = cv2.addWeighted(vis_img, 1, drivable_overlay, 0.4, 0)
        lane_overlay = np.zeros_like(vis_img, dtype=np.uint8)
        lane_overlay[lane_mask == 1] = (255, 0, 0)
        vis_img = cv2.addWeighted(vis_img, 1, lane_overlay, 1, 0)

        # YENİ: Ayarlanabilir ROI çerçevesini sarı renkte çiz
        cv2.polylines(vis_img, [roi_vertices], isClosed=True, color=(0, 255, 255), thickness=3)

        return vis_img

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()