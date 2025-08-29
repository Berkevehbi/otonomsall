#
# line_detector_pkg/line_detector_pkg/line_detector_node.py
#
# HUNCAR 2025 – Adaptif Şerit & Yol Kenarı Tespiti
# Güncellenme: 24 Tem 2025
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        # Parametreler
        self.declare_parameter('camera_topic',
                               '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('publish_bev_image', True)
        self.declare_parameter('detection_mode', 'hough')  # 'hough' | 'poly'
        self.declare_parameter('source_points',
                               [550, 450, 730, 450, 200, 720, 1080, 720])
        self.declare_parameter('destination_size', [300, 400])

        # Kuş-bakışı dönüşümü için matris
        src = np.float32(self.get_parameter('source_points').value).reshape(4, 2)
        dst_w, dst_h = self.get_parameter('destination_size').value
        dst = np.float32([[0, 0], [dst_w, 0], [0, dst_h], [dst_w, dst_h]])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.MPX = 3.0 / dst_w
        self.MPY = 4.0 / dst_h

        # Abonelik & yayıncılar
        cam_topic = self.get_parameter('camera_topic').value
        self.bridge = CvBridge()
        self.create_subscription(Image, cam_topic, self.image_cb, 10)
        self.pub_lines = self.create_publisher(MarkerArray,
                                               '/camera/detected_lines_metric',
                                               10)
        if self.get_parameter('publish_bev_image').value:
            self.pub_bev = self.create_publisher(Image,
                                                 '~/bev_image_with_lines', 10)

        self.get_logger().info(f'LineDetector ready, listening to {cam_topic}')

    # ------------------------ Callbacks ------------------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        bev = cv2.warpPerspective(frame, self.M,
                                  tuple(self.get_parameter(
                                        'destination_size').value))

        mode = self.get_parameter('detection_mode').value
        if mode == 'poly':
            lines_px = self.sliding_window_poly(bev)
        else:
            lines_px = self.hough_detect(bev)

        markers = self.lines_to_markerarray(lines_px)
        self.pub_lines.publish(markers)

        if hasattr(self, 'pub_bev'):
            bev_out = bev.copy()
            for (x1, y1, x2, y2) in lines_px:
                cv2.line(bev_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            self.pub_bev.publish(self.bridge.cv2_to_imgmsg(bev_out,
                                                           encoding='bgr8'))

    # ------------------------ Adaptif Kenar Algılama ------------------------
    @staticmethod
    def auto_canny(img_gray: np.ndarray,
                   sigma: float = 0.33) -> np.ndarray:
        """
        Medyan-temelli otomatik Canny alt/üst eşik hesaplanır.
        """
        v = np.median(img_gray)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        return cv2.Canny(img_gray, lower, upper)

    @staticmethod
    def hough_detect(img) -> list:
        """
        Adaptif eşik + otomatik Canny ile Hough çizgi tespiti.
        """
        # 1) Gri ton + kontrast dengeleme (CLAHE)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_eq = clahe.apply(gray)

        # 2) Adaptif Gaussian eşikleme
        bin_img = cv2.adaptiveThreshold(
            gray_eq, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 11, 2)

        # 3) Dinamik Canny
        edges = LineDetector.auto_canny(bin_img)

        # 4) HoughLinesP
        hough = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=40, minLineLength=40, maxLineGap=25)

        return [l[0] for l in hough] if hough is not None else []

    # ------------------------ Sliding-Window Polinom (Opsiyonel) ------------------------
    def sliding_window_poly(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binary = (gray > 200).astype(np.uint8) * 255
        histogram = np.sum(binary[binary.shape[0]//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows, margin, minpix = 10, 25, 50
        window_height = binary.shape[0] // nwindows
        nonzero = binary.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        leftx_current, rightx_current = leftx_base, rightx_base
        left_lane_inds, right_lane_inds = [], []

        for window in range(nwindows):
            win_y_low = binary.shape[0] - (window+1)*window_height
            win_y_high = binary.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left = ((nonzeroy >= win_y_low) &
                         (nonzeroy < win_y_high) &
                         (nonzerox >= win_xleft_low) &
                         (nonzerox < win_xleft_high)).nonzero()[0]
            good_right = ((nonzeroy >= win_y_low) &
                          (nonzeroy < win_y_high) &
                          (nonzerox >= win_xright_low) &
                          (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.extend(good_left)
            right_lane_inds.extend(good_right)

            if len(good_left) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left]))
            if len(good_right) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right]))

        leftx = nonzerox[left_lane_inds]; lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]; righty = nonzeroy[right_lane_inds]

        lines = []
        for x, y in zip([leftx, rightx], [lefty, righty]):
            if len(x) < 4: continue
            fit = np.polyfit(y, x, 2)
            ploty = np.linspace(0, binary.shape[0]-1, 20)
            plotx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
            for i in range(len(ploty)-1):
                lines.append((int(plotx[i]), int(ploty[i]),
                              int(plotx[i+1]), int(ploty[i+1])))
        return lines

    # ------------------------ MarkerArray Çevirici ------------------------
    def lines_to_markerarray(self, lines_px):
        center_x_px = self.get_parameter('destination_size').value[0] / 2.0
        markers = MarkerArray()
        for idx, (x1, y1, x2, y2) in enumerate(lines_px):
            x1_m = (self.get_parameter(
                    'destination_size').value[1] - y1) * self.MPY
            y1_m = (center_x_px - x1) * self.MPX
            x2_m = (self.get_parameter(
                    'destination_size').value[1] - y2) * self.MPY
            y2_m = (center_x_px - x2) * self.MPX

            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detected_lines_metric'
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            m.points = [Point(x=x1_m, y=y1_m, z=0.0),
                        Point(x=x2_m, y=y2_m, z=0.0)]
            markers.markers.append(m)
        return markers

def main():
    rclpy.init()
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
