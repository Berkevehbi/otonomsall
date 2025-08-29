#!/usr/bin/env python3
import sys
import os

# PATH düzeltme (lib/ importu için)
package_source_path = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, package_source_path)
parent_dir = os.path.dirname(package_source_path)
lib_dir = os.path.join(parent_dir, "lib")
if os.path.exists(lib_dir):
    sys.path.insert(0, lib_dir)

# Model importları
from lib.config import cfg
from lib.config import update_config
from lib.models import get_net

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
import time
import torchvision.transforms as transforms

from smart_can_msgs.msg import (
    Rcthrtdata as RC_THRT_DATA,
    Autonomousbrakepedalcontrol as AUTONOMOUS_BrakePedalControl,
    Rcunittoomux as rc_unittoOmux,
    Autonomoushbmotorcontrol as AUTONOMOUS_HB_MotorControl,
    Autonomoussteeringmotcontrol as AUTONOMOUS_SteeringMot_Control
)

# === PARAMETRELER ===
WEIGHTS_PATH = "/root/robotaxi_ws/install/robotaxi_perception/share/robotaxi_perception/weights/End-to-end.pth"
CONFIG_PATH  = "/root/robotaxi_ws/install/robotaxi_perception/share/robotaxi_perception/lib/config/bdd100k.yaml"
IMG_SIZE     = 640
DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

# === MODEL LOAD ===
def load_perception_model(weights_path, config_path, device):
    class Args:
        cfg = config_path
        modelDir, logDir, dataDir, prevModelDir = '', '', '', ''
    args = Args()
    update_config(cfg, args)
    model = get_net(cfg)
    try:
        checkpoint = torch.load(weights_path, map_location=device, weights_only=True)
    except:
        checkpoint = torch.load(weights_path, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint['state_dict'])
    model.to(device)
    model.eval()
    return model

class SimpleLaneFollower(Node):
    def __init__(self):
        super().__init__('simple_lane_follower')

        # CAN
        self.thrt_pub      = self.create_publisher(RC_THRT_DATA, '/beemobs/RC_THRT_DATA', 10)
        self.brake_pub     = self.create_publisher(AUTONOMOUS_BrakePedalControl, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.unittomux_pub = self.create_publisher(rc_unittoOmux, '/beemobs/rc_unittoOmux', 10)
        self.hb_pub        = self.create_publisher(AUTONOMOUS_HB_MotorControl, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.steering_pub  = self.create_publisher(AUTONOMOUS_SteeringMot_Control, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)

        # Sensorlar
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.bridge = CvBridge()
        self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.camera_callback, 10)

        self.create_timer(0.05, self.control_loop)  # 20Hz

        # State
        self.obstacle_close = False
        self.front_min = 99.0
        self.ignition_sent = False

        # Şerit takibi ve PID
        self.offset_m = 0.0
        self.offset_hist = [0.0]
        self.pid_last_err = 0.0
        self.pid_int = 0.0
        self.last_time = time.time()

        # PID parametreleri (TEST İÇİN BUNLARI OYNAYABİLİRSİN)
        self.kp = 0.6
        self.ki = 0.0
        self.kd = 0.09

        # Model
        self.device = DEVICE
        self.get_logger().info(f"Model cihazı: {self.device}")
        self.model = load_perception_model(WEIGHTS_PATH, CONFIG_PATH, self.device)
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((IMG_SIZE, IMG_SIZE)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        self.get_logger().info("🚦 Simple Lane Follower node başlatıldı ve model yüklendi.")

    def steer_rad_to_pwm(self, steer_cmd):
        pwm = int(127 + steer_cmd * 127)
        return pwm

    def lidar_callback(self, msg):
        n = len(msg.ranges)
        center = n // 2
        window = 30
        front_ranges = msg.ranges[center - window:center + window]
        front_ranges = [r for r in front_ranges if 0.1 < r < float('inf')]
        if not front_ranges:
            self.obstacle_close = False
            self.front_min = 99.0
            return
        self.front_min = min(front_ranges)
        self.obstacle_close = self.front_min < 10.0

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CV Bridge Hatası: {e}")
            return

        img_input = cv2.resize(cv_image, (IMG_SIZE, IMG_SIZE))
        img_tensor = self.transform(img_input).to(self.device).unsqueeze(0)
        with torch.no_grad():
            _, _, ll_seg_out = self.model(img_tensor)
        _, ll_seg_mask = torch.max(ll_seg_out, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy().astype(np.uint8)
        ll_seg_mask = cv2.resize(ll_seg_mask, (cv_image.shape[1], cv_image.shape[0]))

        h, w = ll_seg_mask.shape
        roi_mask = np.zeros_like(ll_seg_mask)
        p1 = (200, h)
        p2 = (w-200, h)
        p3 = (int(0.75*w), int(0.4*h))
        p4 = (int(0.25*w), int(0.4*h))
        roi_vertices = np.array([[p1, p2, p3, p4]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        ll_seg_mask_roi = cv2.bitwise_and(ll_seg_mask, ll_seg_mask, mask=roi_mask)

        points = np.column_stack(np.where(ll_seg_mask_roi > 0))
        if len(points) == 0:
            self.offset_m = 0.0
            return
        min_y = int(h * 0.66)
        base_points = points[points[:,0] > min_y]
        if len(base_points) == 0:
            self.offset_m = 0.0
            return
        mean_x = np.mean(base_points[:,1])
        image_center = w // 2
        px_per_meter = 250  # Gözleme göre ayarla!
        self.offset_m = (mean_x - image_center) / px_per_meter
        self.offset_hist.append(self.offset_m)
        if len(self.offset_hist) > 6:
            self.offset_hist.pop(0)
        self.offset_m = np.mean(self.offset_hist)
        self.get_logger().info(f"Offset (ortalama): {self.offset_m:.2f} m")

    def control_loop(self):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.05
        self.last_time = now

        if not self.ignition_sent:
            self.send_ignition_and_gear()
            time.sleep(1)
            self.release_el_freni()
            self.ignition_sent = True

        # ENGEL VARSA DUR!
        if self.obstacle_close:
            self.send_brake(True, 100)
            self.send_throttle(0)
            return

        # ŞERİT TAKİBİ VE PID DİREKSİYON
        error = -self.offset_m
        self.pid_int += error * dt
        d_error = (error - self.pid_last_err) / dt if dt > 0 else 0.0
        self.pid_last_err = error
        steer_cmd = self.kp * error + self.ki * self.pid_int + self.kd * d_error
        steer_cmd = np.clip(steer_cmd, -1.0, 1.0)  # (-1: sol, 1: sağ)

        # LOG
        self.get_logger().info(f"PID: error={error:.2f} int={self.pid_int:.2f} der={d_error:.2f} steer_cmd={steer_cmd:.2f}")

        # --- GAZ HER ZAMAN 70 ---
        self.send_brake(False)
        self.send_throttle(70)
        self.send_steering(steer_cmd)

    def send_throttle(self, position):
        msg = RC_THRT_DATA()
        msg.rc_thrt_pedal_position = position
        msg.rc_thrt_pedal_press = 0 if position > 0 else 1
        self.thrt_pub.publish(msg)

    def send_brake(self, enable, pwm=0):
        msg = AUTONOMOUS_BrakePedalControl()
        msg.autonomous_brakemotor_voltage = 1
        msg.autonomous_brakepedalmotor_per = pwm if enable else 0
        msg.autonomous_brakepedalmotor_acc = 10000 if enable else 0
        msg.autonomous_brakepedalmotor_en = 1 if enable else 0
        self.brake_pub.publish(msg)

    def send_steering(self, steer_cmd_rad):
        pwm = self.steer_rad_to_pwm(steer_cmd_rad)
        msg = AUTONOMOUS_SteeringMot_Control()
        msg.autonomous_steeringmot_pwm = int(np.clip(pwm, 0, 255))
        msg.autonomous_steeringmot_en = 1
        self.steering_pub.publish(msg)

    def send_ignition_and_gear(self):
        msg = rc_unittoOmux()
        msg.rc_ignition = 1
        msg.rc_selectiongear = 1
        msg.autonomous_emergency = 0
        self.unittomux_pub.publish(msg)

    def release_el_freni(self):
        msg = AUTONOMOUS_HB_MotorControl()
        msg.autonomous_hb_motor_pwm = 200
        msg.autonomous_hb_motstate = 1
        msg.autonomous_hb_moten = 1
        self.hb_pub.publish(msg)

    def shutdown_procedure(self):
        self.send_throttle(0)
        self.send_brake(True, 100)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_procedure()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
