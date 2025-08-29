import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from time import time

from collections import deque

# Curvature threshold (düz yol için): px cinsinden
CURVATURE_MAX_THRESHOLD = 50000  

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info('Akıllı Şerit Planlayıcı Başlatıldı.')

        # Parametreler
        self.declare_parameter('roi_height_ratio', 0.50)
        self.declare_parameter('fit_lower_ratio', 0.5)
        self.declare_parameter('dbscan_eps', 40)
        self.declare_parameter('dbscan_min_samples', 65)
        self.declare_parameter('min_cluster_points', 60)
        self.declare_parameter('jump_limit_px', 50)
        self.declare_parameter('smoothing_window_size', 5)
        self.declare_parameter('default_lane_width_px', 400)
        self.declare_parameter('min_lane_width_px', 400)
        self.declare_parameter('max_lane_width_px', 800)
        self.declare_parameter('max_points_for_fit', 4000)
        self.declare_parameter('lane_center_eval_ratio', 0.75)

        # __init__ içinde mevcut declare_parameter'ların altına ekle:
        self.declare_parameter('use_morph_close', True)
        self.declare_parameter('morph_kernel_w', 3)
        self.declare_parameter('morph_kernel_h', 15)
        self.declare_parameter('morph_iterations', 1)

        self.declare_parameter('dbscan_y_scale', 0.4)   # y'yi sıkıştırma katsayısı (0.3–0.6 arası deneyebilirsin)
        self.declare_parameter('side_select_window_px', 40)  # y_eval etrafında ±pencere
        self.declare_parameter('side_select_min_pts', 20)    # pencere içinde min nokta

        # Faz 3: ayrı kenar KFlari ve kaybolma yönetimi
        self.declare_parameter('edge_kf_enable', True)
        self.declare_parameter('edge_kf_R_base', 4.0)     # yüksek güven ölçüm için R
        self.declare_parameter('edge_kf_R_max', 30.0)     # düşük güven ölçüm için R
        self.declare_parameter('edge_kf_Q_var', 0.1)      # süreç gürültüsü (pos-vel modeli)
        self.declare_parameter('lost_warn_frames', 5)     # bu kadar kare kayıp → uyarı modu
        self.declare_parameter('lost_hard_frames', 15)    # bu kadar kare kayıp → agresif fallback

        
        # YENİ: Metreye çevirme parametrelerini aktif et
        self.declare_parameter('ym_per_pix', 30/720) # dikeyde metre/piksel
        self.declare_parameter('xm_per_pix', 3.7/700) # yatayda metre/piksel

        # Merkez stabilizasyonu (history + slew)
        self.declare_parameter('center_hist_len', 10)        # son N gözlem
        self.declare_parameter('center_stat', 'median')      # 'median' | 'mean' | 'trimmed'
        self.declare_parameter('center_trim_ratio', 0.2)     # trimmed-mean için (alt/üst %20 kırp)
        self.declare_parameter('center_slew_px', 8)          # kare başına maks merkez değişimi (px)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/perception/lane_mask',
            self.mask_callback,
            10)
        
        # MEVCUT PİKSEL PUBLISHER'I KORUYORUZ
        self.offset_publisher = self.create_publisher(Float32, '/planning/lane_offset_px', 10)

        # YENİ METRE PUBLISHER'LARINI EKLİYORUZ
        self.offset_publisher_m = self.create_publisher(Float32, '/planning/lane_offset_m', 10)
        self.curvature_publisher = self.create_publisher(Float32, '/planning/lane_curvature_radius_m', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/planning/debug_image', 10)

        self.last_valid_offset = 0.0
        self.offset_window = []

        self.ema_left  = None
        self.ema_right = None
        self.ema_alpha = 0.2

        self.kf = self.initialize_kalman_filter()
        self.last_update_time = None

        self.debug_last_log_time = time()

        self.get_logger().info('Planlayıcı hazır. Şerit maskesi bekleniyor...')

        # __init__ sonlarına doğru, mevcut alanların yanına ekle:
        self.width_ema = self.get_parameter('default_lane_width_px').get_parameter_value().integer_value
        self.width_alpha = 0.2  # EMA gain
        self.last_good_center_px = None

        # Kenar bazlı KF'ler (x_base@y_eval için)
        self.kf_left  = self._init_edge_kf()
        self.kf_right = self._init_edge_kf()
        self.edge_last_time = None  # kenar KFlari için zaman

        # Kaybolma sayaçları
        self.lost_left_frames  = 0
        self.lost_right_frames = 0

        L = self.get_parameter('center_hist_len').get_parameter_value().integer_value
        self.hist_left  = deque(maxlen=L)
        self.hist_right = deque(maxlen=L)

    def estimate_x_at_y(self, cluster, y_eval, window_px=40, min_pts=20):
        """
        Bir cluster için y_eval çevresinde (±window_px) x'in medyanını döndürür.
        Yeterli nokta yoksa cluster mean_x'e fallback yapar.
        """
        if cluster is None or len(cluster) == 0:
            return None
        y = cluster[:, 0]
        x = cluster[:, 1]
        mask = np.abs(y - y_eval) <= window_px
        if np.count_nonzero(mask) >= min_pts:
            return float(np.median(x[mask]))
        # fallback: tüm kümenin mean_x'i
        return float(np.mean(x))
    
    def _robust_stat(self, values, stat='median', trim_ratio=0.2):
        if values is None or len(values) == 0:
            return None
        arr = np.asarray(values, dtype=np.float32)
        if stat == 'median':
            return float(np.median(arr))
        elif stat == 'trimmed':
            if len(arr) < 3:
                return float(np.mean(arr))
            r = max(0.0, min(0.45, float(trim_ratio)))
            k = int(len(arr) * r)
            arr_sorted = np.sort(arr)
            return float(np.mean(arr_sorted[k: len(arr_sorted)-k])) if (len(arr_sorted) - 2*k) > 0 else float(np.mean(arr))
        else:  # 'mean'
            return float(np.mean(arr))

    def _init_edge_kf(self):
        """x_base@y_eval için 1D konum+hız (2D durum) KF"""
        kf = KalmanFilter(dim_x=2, dim_z=1)
        dt = 1.0/30.0
        kf.x = np.array([[0.],[0.]])        # [x, vx]
        kf.F = np.array([[1., dt],
                        [0., 1.]])
        kf.H = np.array([[1., 0.]])
        kf.P *= 500.0                       # başlangıç belirsizliği
        # Q ve R run-time'da güncellenecek
        return kf

    def _edge_confidence(self, cluster, yspan, deg, y_eval, side_win, side_min_pts, h, fit_lower_ratio):
        """
        0..1 arası bir güven skorunu heuristik olarak hesaplar.
        Faktörler: y_eval çevresindeki nokta sayısı, dikey kapsama, fit derecesi.
        """
        if cluster is None or len(cluster) == 0:
            return 0.0
        y = cluster[:,0]; x = cluster[:,1]
        mask = np.abs(y - y_eval) <= side_win
        n_eval = int(np.count_nonzero(mask))
        # y kapsama: fit bandının yüksekliği ~ (h - h*fit_lower_ratio)
        band_h = max(1, int(h * (1 - fit_lower_ratio)))
        yspan_norm = np.clip(yspan / band_h, 0.0, 1.0)
        # deg2 fit biraz ödüllü
        deg_score = 1.0 if deg == 2 else (0.7 if deg == 1 else 0.4)
        eval_score = np.clip(n_eval / max(1, side_min_pts*1.5), 0.0, 1.0)
        # ağırlıklı kombinasyon
        conf = 0.5*eval_score + 0.3*yspan_norm + 0.2*deg_score
        return float(np.clip(conf, 0.0, 1.0))


    # YENİ YARDIMCI FONKSİYON (class'ın içine ekle)
    def initialize_kalman_filter(self):
        self.get_logger().info("Kalman Filtresi başlatılıyor...")
        kf = KalmanFilter(dim_x=2, dim_z=1)
        
        # State (Durum) Vektörü: [offset, offset_hızı]
        # Başlangıçta 0 ofset ve 0 hızda olduğumuzu varsayıyoruz.
        kf.x = np.array([[0.], [0.]])

        # State Transition (Durum Geçiş) Matrisi F
        # Bir sonraki durumun nasıl hesaplanacağını söyler.
        # pos_new = pos_old + vel_old * dt
        # vel_new = vel_old
        dt = 1.0 / 30.0 # Varsayılan zaman adımı (saniye)
        kf.F = np.array([[1., dt],
                        [0., 1.]])

        # Measurement (Ölçüm) Fonksiyonu H
        # Durum vektöründen ölçümümüzü (sadece ofset) nasıl alacağımızı söyler.
        kf.H = np.array([[1., 0.]])

        # Covariance (Kovaryans) Matrisi P
        # Tahminlerimize ne kadar güvendiğimizi belirtir. Başlangıçta çok emin değiliz.
        kf.P *= 1000.

        # Measurement Noise (Ölçüm Gürültüsü) R
        # Şerit tespitinden gelen verinin ne kadar "gürültülü" olduğunu belirtir.
        # Bu değeri artırırsan filtre yeni ölçümlere daha az güvenir. (ANA AYAR PARAMETRESİ)
        kf.R = np.array([[5]]) # Tuning parametresi

        # Process Noise (Süreç Gürültüsü) Q
        # Hareket modelimizin ne kadar "mükemmel" olduğunu belirtir.
        # Bu değeri artırırsan filtre yeni ölçümlere daha hızlı adapte olur. (ANA AYAR PARAMETRESİ)
        kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1) # Tuning parametresi

        return kf
    
    def is_cluster_in_image(self, cluster, w, min_inlier_ratio=0.4):
        if cluster is None or len(cluster) == 0:
            return False
        inliers = np.logical_and(0 <= cluster[:,1], cluster[:,1] < w)
        ratio = np.sum(inliers) / len(cluster)
        return ratio > min_inlier_ratio

    def mask_callback(self, msg: Image):
        t0 = time()
        try:
            lane_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        h, w = lane_mask.shape
        vehicle_center_x_px = w // 2

        # --- Param cache ---
        roi_height_ratio   = self.get_parameter('roi_height_ratio').get_parameter_value().double_value
        fit_lower_ratio    = self.get_parameter('fit_lower_ratio').get_parameter_value().double_value
        dbscan_eps         = self.get_parameter('dbscan_eps').get_parameter_value().integer_value
        dbscan_min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value
        min_cluster_points = self.get_parameter('min_cluster_points').get_parameter_value().integer_value
        max_points_for_fit = self.get_parameter('max_points_for_fit').get_parameter_value().integer_value
        lane_eval_ratio    = self.get_parameter('lane_center_eval_ratio').get_parameter_value().double_value
        default_lane_width = self.get_parameter('default_lane_width_px').get_parameter_value().integer_value
        min_width          = self.get_parameter('min_lane_width_px').get_parameter_value().integer_value
        max_width          = self.get_parameter('max_lane_width_px').get_parameter_value().integer_value
        jump_limit         = self.get_parameter('jump_limit_px').get_parameter_value().integer_value
        xm_per_pix         = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        ym_per_pix         = self.get_parameter('ym_per_pix').get_parameter_value().double_value

        # Faz 2
        use_morph_close = self.get_parameter('use_morph_close').get_parameter_value().bool_value if self.has_parameter('use_morph_close') else True
        mkw             = self.get_parameter('morph_kernel_w').get_parameter_value().integer_value if self.has_parameter('morph_kernel_w') else 3
        mkh             = self.get_parameter('morph_kernel_h').get_parameter_value().integer_value if self.has_parameter('morph_kernel_h') else 15
        morph_iter      = self.get_parameter('morph_iterations').get_parameter_value().integer_value if self.has_parameter('morph_iterations') else 1
        dbscan_y_scale  = self.get_parameter('dbscan_y_scale').get_parameter_value().double_value if self.has_parameter('dbscan_y_scale') else 0.4
        side_win        = self.get_parameter('side_select_window_px').get_parameter_value().integer_value if self.has_parameter('side_select_window_px') else 40
        side_min_pts    = self.get_parameter('side_select_min_pts').get_parameter_value().integer_value if self.has_parameter('side_select_min_pts') else 20

        # Faz 3
        edge_kf_enable  = self.get_parameter('edge_kf_enable').get_parameter_value().bool_value if self.has_parameter('edge_kf_enable') else True
        edge_R_base     = self.get_parameter('edge_kf_R_base').get_parameter_value().double_value if self.has_parameter('edge_kf_R_base') else 4.0
        edge_R_max      = self.get_parameter('edge_kf_R_max').get_parameter_value().double_value if self.has_parameter('edge_kf_R_max') else 30.0
        edge_Q_var      = self.get_parameter('edge_kf_Q_var').get_parameter_value().double_value if self.has_parameter('edge_kf_Q_var') else 0.1
        lost_warn       = self.get_parameter('lost_warn_frames').get_parameter_value().integer_value if self.has_parameter('lost_warn_frames') else 5
        lost_hard       = self.get_parameter('lost_hard_frames').get_parameter_value().integer_value if self.has_parameter('lost_hard_frames') else 15

        # History + Slew
        center_hist_len  = self.get_parameter('center_hist_len').get_parameter_value().integer_value if self.has_parameter('center_hist_len') else 10
        center_stat      = self.get_parameter('center_stat').get_parameter_value().string_value if self.has_parameter('center_stat') else 'median'
        center_trim      = self.get_parameter('center_trim_ratio').get_parameter_value().double_value if self.has_parameter('center_trim_ratio') else 0.2
        center_slew_px   = self.get_parameter('center_slew_px').get_parameter_value().integer_value if self.has_parameter('center_slew_px') else 8

        # History deques (guard)
        if not hasattr(self, 'hist_left') or not isinstance(self.hist_left, deque):
            self.hist_left = deque(maxlen=center_hist_len)
        if not hasattr(self, 'hist_right') or not isinstance(self.hist_right, deque):
            self.hist_right = deque(maxlen=center_hist_len)

        # --- STEP 1: ROI ---
        roi_start = int(h * (1 - roi_height_ratio))
        roi_mask  = lane_mask[roi_start:, :]
        t1 = time()

        # --- STEP 2: Points (morph close opsiyonel) ---
        if use_morph_close:
            roi_bin = (roi_mask > 0).astype(np.uint8) * 255
            kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (max(1, mkw), max(1, mkh)))
            roi_proc = cv2.morphologyEx(roi_bin, cv2.MORPH_CLOSE, kernel, iterations=max(1, morph_iter))
            points = np.column_stack(np.where(roi_proc > 0))
            if len(points) == 0:
                points = np.column_stack(np.where(roi_mask > 0))
        else:
            points = np.column_stack(np.where(roi_mask > 0))

        pts_raw = len(points)
        if pts_raw == 0:
            dbg = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
            cv2.line(dbg, (0, roi_start), (w, roi_start), (0, 128, 255), 2)
            cv2.putText(dbg, "No lane pixels in ROI", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.line(dbg, (vehicle_center_x_px, h), (vehicle_center_x_px, h - 60), (255, 0, 255), 3)
            self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
            self.get_logger().warn("[PP] pts_raw=0 → early return")
            return

        if pts_raw > max_points_for_fit:
            idx = np.random.choice(pts_raw, max_points_for_fit, replace=False)
            points = points[idx]
        pts_used = len(points)

        points[:, 0] += roi_start  # global y
        t2 = time()

        # --- STEP 3: DBSCAN (anizotropik: y sıkıştır) ---
        points_scaled = points.astype(np.float32).copy()
        points_scaled[:, 0] *= float(dbscan_y_scale)
        try:
            clustering = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(points_scaled)
        except Exception as e:
            self.get_logger().error(f'DBSCAN Error: {e}')
            return

        clusters = []
        sizes_all = []
        labels = clustering.labels_
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:
                continue
            cluster = points[labels == label]
            sizes_all.append(len(cluster))
            if len(cluster) > min_cluster_points:
                clusters.append(cluster)

        n_clusters_all = len(unique_labels[unique_labels != -1])
        n_clusters_kept = len(clusters)

        if n_clusters_kept == 0:
            dbg = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
            cv2.line(dbg, (0, roi_start), (w, roi_start), (0, 128, 255), 2)
            cv2.putText(dbg, "No clusters after DBSCAN filter", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.line(dbg, (vehicle_center_x_px, h), (vehicle_center_x_px, h - 60), (255, 0, 255), 3)
            self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
            self.get_logger().warn(f"[PP] pts_raw={pts_raw} pts_used={pts_used} clusters_all={n_clusters_all} kept=0 → early return")
            return
        t3 = time()

        # --- STEP 4: Sol/Sağ seçim (y_eval çevresinde x-medyan) ---
        y_eval_px = int(h * lane_eval_ratio)

        left_cluster = None
        right_cluster = None
        left_min_dist = float('inf')
        right_min_dist = float('inf')

        # ÖNEMLİ: y_eval±side_win penceresinde yeterli nokta yoksa cluster'ı aday yapma
        for cluster in clusters:
            y_vals = cluster[:, 0]
            x_vals = cluster[:, 1]
            win_mask = np.abs(y_vals - y_eval_px) <= side_win
            n_eval = int(np.count_nonzero(win_mask))
            if n_eval < side_min_pts:
                continue  # zayıf cluster'ı tamamen es geç

            # pencerede yeterli nokta varsa medyan x
            x_at_eval = float(np.median(x_vals[win_mask]))
            dist = abs(vehicle_center_x_px - x_at_eval)
            if x_at_eval < vehicle_center_x_px:
                if dist < left_min_dist:
                    left_min_dist = dist
                    left_cluster = cluster
            else:
                if dist < right_min_dist:
                    right_min_dist = dist
                    right_cluster = cluster

        # --- STEP 4.b: Seçilen kenarlarda ikinci güvenlik — pencerede nokta sayısı hala azsa düşür ---
        def _count_eval_pts(cluster, y_eval, win):
            if cluster is None or len(cluster) == 0:
                return 0
            return int(np.count_nonzero(np.abs(cluster[:, 0] - y_eval) <= win))

        nL_eval = _count_eval_pts(left_cluster,  y_eval_px, side_win)
        nR_eval = _count_eval_pts(right_cluster, y_eval_px, side_win)

        if nL_eval < side_min_pts:
            if left_cluster is not None:
                self.get_logger().warn(f"Left edge weak near y={y_eval_px}: n={nL_eval} < {side_min_pts} → mark as NOT DETECTED")
            left_cluster = None
        if nR_eval < side_min_pts:
            if right_cluster is not None:
                self.get_logger().warn(f"Right edge weak near y={y_eval_px}: n={nR_eval} < {side_min_pts} → mark as NOT DETECTED")
            right_cluster = None

        yspan_L = int(left_cluster[:, 0].ptp())  if left_cluster  is not None and len(left_cluster)  else 0
        yspan_R = int(right_cluster[:, 0].ptp()) if right_cluster is not None and len(right_cluster) else 0

        # --- STEP 5: Fit points + polynomial ---
        fit_y_limit = int(h * fit_lower_ratio)
        left_fit_points_px  = self.filter_fit_points(left_cluster,  fit_y_limit)
        right_fit_points_px = self.filter_fit_points(right_cluster, fit_y_limit)

        left_coeffs_px_deg2  = self.fit_polynomial(left_fit_points_px,  degree=2)
        right_coeffs_px_deg2 = self.fit_polynomial(right_fit_points_px, degree=2)

        # --- STEP 6: Width sanity + fallback ---
        is_sane = self.is_lane_width_sane(left_coeffs_px_deg2, right_coeffs_px_deg2, h)
        left_coeffs_px  = left_coeffs_px_deg2  if is_sane else self.fit_polynomial(left_fit_points_px,  degree=1)
        right_coeffs_px = right_coeffs_px_deg2 if is_sane else self.fit_polynomial(right_fit_points_px, degree=1)
        deg_left  = 2 if (is_sane and left_coeffs_px_deg2  is not None) else (1 if left_coeffs_px  is not None else 0)
        deg_right = 2 if (is_sane and right_coeffs_px_deg2 is not None) else (1 if right_coeffs_px is not None else 0)

        # --- STEP 7: EMA smoothing on coeffs ---
        final_left_coeffs_px  = self.smooth_coeffs(left_coeffs_px,  side='left')
        final_right_coeffs_px = self.smooth_coeffs(right_coeffs_px, side='right')

        # --- STEP 8: Evaluate bases at y_eval ---
        x_left_base_px  = self.poly_eval(final_left_coeffs_px,  y_eval_px) if final_left_coeffs_px  is not None else None
        x_right_base_px = self.poly_eval(final_right_coeffs_px, y_eval_px) if final_right_coeffs_px is not None else None

        if x_left_base_px  is not None and not (0 <= x_left_base_px  < w): x_left_base_px  = None
        if x_right_base_px is not None and not (0 <= x_right_base_px < w): x_right_base_px = None

        # --- STEP 10: Center & width_ema (temel) ---
        has_left  = x_left_base_px  is not None
        has_right = x_right_base_px is not None
        confidence = (1.0 if has_left else 0.0) + (1.0 if has_right else 0.0)

        if has_left and has_right:
            curr_width = abs(x_right_base_px - x_left_base_px)
            curr_width = max(min_width, min(max_width, curr_width))
            self.width_ema = self.width_alpha * curr_width + (1 - self.width_alpha) * self.width_ema
            center_mode = "both"
            lane_center_x_px = 0.5 * (x_left_base_px + x_right_base_px)
        elif has_left:
            center_mode = "left-only"
            lane_center_x_px = x_left_base_px + self.width_ema / 2.0
            self.get_logger().warn('Only left lane detected → estimating right using width_ema.')
        elif has_right:
            center_mode = "right-only"
            lane_center_x_px = x_right_base_px - self.width_ema / 2.0
            self.get_logger().warn('Only right lane detected → estimating left using width_ema.')
        else:
            center_mode = "none"
            lane_center_x_px = self.last_good_center_px if self.last_good_center_px is not None else vehicle_center_x_px
            self.get_logger().warn('No lane detected → holding last center (or vehicle center if first frame).')

        # --- STEP 8.5: Kenar KF'leri (Faz 3) ---
        if not hasattr(self, 'kf_left'):   self.kf_left  = self._init_edge_kf()
        if not hasattr(self, 'kf_right'):  self.kf_right = self._init_edge_kf()
        if not hasattr(self, 'edge_last_time'): self.edge_last_time = None
        if not hasattr(self, 'lost_left_frames'):  self.lost_left_frames = 0
        if not hasattr(self, 'lost_right_frames'): self.lost_right_frames = 0

        kfL_mode = kfR_mode = "off"
        confL = confR = 0.0

        if edge_kf_enable:
            now = time()
            dt_edge = (1.0/30.0) if (self.edge_last_time is None) else max(1e-3, now - self.edge_last_time)
            self.edge_last_time = now

            for kf in (self.kf_left, self.kf_right):
                kf.F[0,1] = dt_edge
                kf.Q = Q_discrete_white_noise(dim=2, dt=dt_edge, var=edge_Q_var)

            # Sol
            if has_left:
                if hasattr(self, '_edge_confidence'):
                    confL = self._edge_confidence(left_cluster, yspan_L, deg_left, y_eval_px,
                                                side_win, side_min_pts, h, fit_lower_ratio)
                else:
                    confL = 1.0
                R_L = edge_R_base + (edge_R_max - edge_R_base) * (1.0 - confL)
                self.kf_left.predict()
                self.kf_left.R = np.array([[R_L]])
                self.kf_left.update(np.array([[x_left_base_px]]))
                self.lost_left_frames = 0
                kfL_mode = f"upd({R_L:.1f})"
            else:
                self.kf_left.predict()
                self.lost_left_frames += 1
                kfL_mode = "pred"

            # Sağ
            if has_right:
                if hasattr(self, '_edge_confidence'):
                    confR = self._edge_confidence(right_cluster, yspan_R, deg_right, y_eval_px,
                                                side_win, side_min_pts, h, fit_lower_ratio)
                else:
                    confR = 1.0
                R_R = edge_R_base + (edge_R_max - edge_R_base) * (1.0 - confR)
                self.kf_right.predict()
                self.kf_right.R = np.array([[R_R]])
                self.kf_right.update(np.array([[x_right_base_px]]))
                self.lost_right_frames = 0
                kfR_mode = f"upd({R_R:.1f})"
            else:
                self.kf_right.predict()
                self.lost_right_frames += 1
                kfR_mode = "pred"

            # KF tahminleri
            xL_hat = float(self.kf_left.x[0,0])
            xR_hat = float(self.kf_right.x[0,0])
            width_hat = abs(xR_hat - xL_hat)
            width_ok  = (min_width < width_hat < max_width)
            in_img    = (0 <= xL_hat < w) and (0 <= xR_hat < w)
            hard_loss = (self.lost_left_frames >= lost_hard) or (self.lost_right_frames >= lost_hard)

            if in_img and width_ok and not hard_loss:
                lane_center_x_px_kf = 0.5 * (xL_hat + xR_hat)
                lane_center_x_px = 0.7 * lane_center_x_px_kf + 0.3 * lane_center_x_px
                center_mode += "|kf"

        # --- STEP 10.b: History + Slew stabilizasyonu ---
        def _robust_stat_local(values, stat='median', trim_ratio=0.2):
            if values is None or len(values) == 0:
                return None
            arr = np.asarray(values, dtype=np.float32)
            if stat == 'median':
                return float(np.median(arr))
            elif stat == 'trimmed':
                if len(arr) < 3:
                    return float(np.mean(arr))
                r = max(0.0, min(0.45, float(trim_ratio)))
                k = int(len(arr) * r)
                arr_sorted = np.sort(arr)
                return float(np.mean(arr_sorted[k: len(arr_sorted)-k])) if (len(arr_sorted) - 2*k) > 0 else float(np.mean(arr))
            else:
                return float(np.mean(arr))

        # history'lere ekle
        if has_left:  self.hist_left.append(float(x_left_base_px))
        if has_right: self.hist_right.append(float(x_right_base_px))

        # stabilize kenarlar
        if hasattr(self, '_robust_stat'):
            xL_stab = self._robust_stat(self.hist_left,  stat=center_stat, trim_ratio=center_trim) if len(self.hist_left)>0  else (float(x_left_base_px)  if has_left  else None)
            xR_stab = self._robust_stat(self.hist_right, stat=center_stat, trim_ratio=center_trim) if len(self.hist_right)>0 else (float(x_right_base_px) if has_right else None)
        else:
            xL_stab = _robust_stat_local(self.hist_left,  stat=center_stat, trim_ratio=center_trim) if len(self.hist_left)>0  else (float(x_left_base_px)  if has_left  else None)
            xR_stab = _robust_stat_local(self.hist_right, stat=center_stat, trim_ratio=center_trim) if len(self.hist_right)>0 else (float(x_right_base_px) if has_right else None)

        # history tabanlı merkez
        lane_center_hist = lane_center_x_px
        if (xL_stab is not None) and (xR_stab is not None):
            lane_center_hist = 0.5 * (xL_stab + xR_stab)
        elif xL_stab is not None:
            lane_center_hist = xL_stab + self.width_ema / 2.0
        elif xR_stab is not None:
            lane_center_hist = xR_stab - self.width_ema / 2.0

        # füzyon + slew limiter
        lane_center_x_px = 0.6 * lane_center_hist + 0.4 * lane_center_x_px
        center_mode += "|hist"

        if hasattr(self, 'last_good_center_px') and self.last_good_center_px is not None:
            delta = lane_center_x_px - self.last_good_center_px
            if abs(delta) > center_slew_px:
                lane_center_x_px = self.last_good_center_px + np.sign(delta) * center_slew_px

        # Jump limiter: düşük güvende ani sıçrama → tut
        if self.last_good_center_px is not None and confidence < 1.5:
            if abs(lane_center_x_px - self.last_good_center_px) > jump_limit:
                lane_center_x_px = self.last_good_center_px

        self.last_good_center_px = lane_center_x_px

        offset_px = lane_center_x_px - vehicle_center_x_px
        t4 = time()

        # --- STEP 11: Global offset KF ---
        current_time = time()
        dt = (1.0/30.0) if (self.last_update_time is None) else (current_time - self.last_update_time)
        self.last_update_time = current_time

        self.kf.F[0, 1] = dt
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1)
        self.kf.predict()

        offset_m = offset_px * xm_per_pix
        kf_mode_global = "predict-only"
        try:
            if confidence >= 1.5:
                self.kf.R = np.array([[5]])
                self.kf.update(np.array([[offset_m]]))
                kf_mode_global = "update-strong"
            elif confidence >= 0.5:
                self.kf.R = np.array([[15]])
                self.kf.update(np.array([[offset_m]]))
                kf_mode_global = "update-weak"
            else:
                pass
        except Exception as e:
            self.get_logger().error(f'Kalman update error: {e}')

        smoothed_offset_m = self.kf.x[0, 0]

        # --- STEP 12: Curvature (left) ---
        curvature_radius_m = float('inf')
        if final_left_coeffs_px is not None:
            a_px, b_px, c_px = final_left_coeffs_px
            A_m = a_px * (xm_per_pix / (ym_per_pix**2))
            B_m = b_px * (xm_per_pix / ym_per_pix)
            y_eval_m = y_eval_px * ym_per_pix
            if abs(A_m) > 1e-6:
                curvature_radius_m = ((1 + (2*A_m*y_eval_m + B_m)**2)**1.5) / np.abs(2*A_m)

        # --- STEP 13: Publish ---
        self.publish_offset(offset_px)
        self.publish_results_m(smoothed_offset_m, curvature_radius_m)

        # --- STEP 14: Debug image + overlays ---
        debug_img = self.visualize_planning(
            lane_mask, left_cluster, right_cluster, lane_center_x_px,
            vehicle_center_x_px, smoothed_offset_m, y_eval_px,
            final_left_coeffs_px, final_right_coeffs_px,
            x_left_base_px, x_right_base_px,
            curvature_radius_m, None
        )
        # ROI çizgisi
        cv2.line(debug_img, (0, roi_start), (w, roi_start), (0, 128, 255), 2)

        lane_width_px = abs(x_right_base_px - x_left_base_px) if (has_left and has_right) else None

        lines = [
            f"morph={use_morph_close} k=({mkw}x{mkh}) it={morph_iter} aniso_y={dbscan_y_scale}",
            f"edgeKF: L={ 'off' if not edge_kf_enable else kfL_mode } lost={getattr(self,'lost_left_frames',0)} conf={confL:.2f} | "
            f"R={ 'off' if not edge_kf_enable else kfR_mode } lost={getattr(self,'lost_right_frames',0)} conf={confR:.2f}",
            f"pts_raw={pts_raw} pts_used={pts_used} clusters_all={n_clusters_all} kept={n_clusters_kept} sizes={sizes_all}",
            f"hasL={left_cluster is not None} hasR={right_cluster is not None} yspanL={yspan_L} yspanR={yspan_R}",
            f"degL={deg_left} degR={deg_right} sane={is_sane}",
            f"xL@{y_eval_px}={None if x_left_base_px is None else int(x_left_base_px)} "
            f"xR@{y_eval_px}={None if x_right_base_px is None else int(x_right_base_px)} "
            f"width={'None' if lane_width_px is None else int(lane_width_px)} widthEMA={int(self.width_ema)}",
            f"sideSel: y_eval={y_eval_px}±{side_win} minPts={side_min_pts}",
            f"nL_eval={nL_eval} nR_eval={nR_eval} thr={side_min_pts}",
        ]
        lines.append(f"center_mode={center_mode} off_px={offset_px:.1f} off_m={smoothed_offset_m:.3f} KFglob={kf_mode_global}")
        lines.append(f"hist: L={len(self.hist_left)} R={len(self.hist_right)} stat={center_stat} trim={center_trim:.2f} slew={center_slew_px}px")

        y0 = 25
        for i, txt in enumerate(lines):
            cv2.putText(debug_img, txt, (10, y0 + i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        t_end = time()
        perf = f"dt_ms: roi={(t1-t0)*1000:.1f} pts={(t2-t1)*1000:.1f} db={(t3-t2)*1000:.1f} fit={(t4-t3)*1000:.1f} total={(t_end-t0)*1000:.1f}"
        cv2.putText(debug_img, perf, (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 2)

        try:
            self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')

        now = time()
        if now - self.debug_last_log_time >= 1.0:
            def _fmt(v):
                return "None" if v is None else f"{float(v):.1f}"
            self.get_logger().info(
                f"left_lane_center_px={_fmt(x_left_base_px)}, "
                f"right_lane_center_px={_fmt(x_right_base_px)}, "
                f"lane_center_px={float(lane_center_x_px):.1f} BU KA.\n"
                f" offset_px={float(offset_px):.1f}, "
                f"offset_m={smoothed_offset_m:.3f}, \n"
                f"curvature_radius_m={_fmt(curvature_radius_m)}, "
                f"width_px={_fmt(lane_width_px)}, \n"
                f"width_ema_px={self.width_ema:.1f}, "
            )
            self.debug_last_log_time = now


    def filter_fit_points(self, cluster, fit_y_limit):
        # Cluster'dan sadece alt bölgedeki (y > fit_y_limit) noktaları seç
        if cluster is None or len(cluster) == 0:
            return None
        return cluster[cluster[:,0] >= fit_y_limit]

    # path_planner_node.py -> PathPlannerNode class'ı içine

    def publish_offset(self, offset_px):
        offset_msg = Float32()
        offset_msg.data = offset_px
        self.offset_publisher.publish(offset_msg)
        #self.get_logger().info(f'Offset: {offset_px:.2f} px')

    def publish_results_m(self, offset_m, curvature_m):
        # Metre cinsinden ofseti yayınla
        offset_msg = Float32()
        offset_msg.data = offset_m
        self.offset_publisher_m.publish(offset_msg)

        # Metre cinsinden eğrilik yarıçapını yayınla
        if curvature_m is not None and np.isfinite(curvature_m):
            curv_msg = Float32()
            curv_msg.data = curvature_m
            self.curvature_publisher.publish(curv_msg)
        
        #self.get_logger().info(f'---> Metric Output: Offset={offset_m:.3f} m | Curvature={curvature_m:.0f} m')

    def compute_slope(self, cluster):
        if cluster is None or len(cluster) < 2:
            return 0
        x = cluster[:, 1]
        y = cluster[:, 0]
        A = np.vstack([x, np.ones(len(x))]).T
        m, _ = np.linalg.lstsq(A, y, rcond=None)[0]
        return m

    def fit_polynomial(self, cluster, degree=2):
        if cluster is None or len(cluster) < degree + 1:
            return None
        y = cluster[:, 0].reshape(-1,1)       # bağımsız değişken
        x = cluster[:, 1]                     # hedef
        # poly + RANSAC pipeline
        model = make_pipeline(
            PolynomialFeatures(degree, include_bias=False),
            RANSACRegressor(residual_threshold=5, min_samples=0.5)
        )
        try:
            model.fit(y, x)
            # coeffs sırasıyla [b, a] veya [b,c,a] olabilir; polyfeatures powers_ ile çek:
            coefs = model.named_steps['ransacregressor'].estimator_.coef_
            intercept = model.named_steps['ransacregressor'].estimator_.intercept_
            if degree == 2:
                # x = a*y^2 + b*y + c
                a = coefs[1]
                b = coefs[0]
                c = intercept
                return np.array([a, b, c])
            else:
                # 1. derece fallback
                return np.array([0, coefs[0], intercept])
        except ValueError:
            return None

    def compute_curvature(self, coeffs, y_eval):
        if coeffs is None or len(coeffs) < 3:
            return None
        a, b = coeffs[0], coeffs[1]
        
        # DÜZELTME: 'a' katsayısı sıfıra çok yakınsa, bu bir çizgidir.
        # Çizginin eğrilik yarıçapı sonsuzdur. Sıfıra bölme hatasını önle.
        if abs(a) < 1e-6:
            return float('inf') # Sonsuz yarıçap

        curvature = ((1 + (2*a*y_eval + b)**2 )**1.5) / np.abs(2*a)
        return curvature
    
    def smooth_coeffs(self, new_coeffs, side='left'):
        if new_coeffs is None:
            # Eğer yeni katsayı yoksa, eskisini döndür ki sistem çökmesin.
            return self.ema_left if side == 'left' else self.ema_right

        if side == 'left':
            if self.ema_left is None:
                self.ema_left = new_coeffs
            else:
                self.ema_left = self.ema_alpha*new_coeffs + (1-self.ema_alpha)*self.ema_left
            return self.ema_left
        else: # side == 'right'
            if self.ema_right is None:
                self.ema_right = new_coeffs
            else:
                self.ema_right = self.ema_alpha*new_coeffs + (1-self.ema_alpha)*self.ema_right
            return self.ema_right

    #def draw_polynomial(self, img, coeffs, y_min, color=(0,255,0), thickness=2):
    #    if coeffs is None:
    #        return
    #    h = img.shape[0]
    #    y_vals = np.arange(y_min, h)          # <-- sadece alt bölüm
    #    x_vals = np.polyval(coeffs, y_vals)
    #    pts = np.array([[int(x), int(y)] for x,y in zip(x_vals, y_vals) if 0<=x<img.shape[1]])
    #    for i in range(1, len(pts)):
    #        cv2.line(img, tuple(pts[i-1]), tuple(pts[i]), color, thickness)

    def draw_polynomial(self, img, coeffs, y_min, color=(0,255,0), thickness=2):
        if coeffs is None:
            return
        h = img.shape[0]
        y_vals = np.arange(y_min, h)
        # x = a*y^2 + b*y + c
        x_vals = self.poly_eval(coeffs, y_vals) # <-- BURAYI YENİ FONKSİYONLA GÜNCELLE
        pts = np.array([[int(x), int(y)] for x,y in zip(x_vals, y_vals) if 0<=x<img.shape[1]])
        # Çizim için noktaları birleştir
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(img, [pts], isClosed=False, color=color, thickness=thickness)

    def poly_eval(self, coeffs, y):
        """
        Katsayıları (a,b,c) olan ve x = a*y^2 + b*y + c şeklinde tanımlanmış
        bir polinomu hesaplar. 1. derece için a=0'dır.
        """
        if coeffs is None:
            return 0
        a, b, c = coeffs
        return a * y**2 + b * y + c

    def is_lane_width_sane(self, left_coeffs, right_coeffs, h):
        """
        Verilen sol ve sağ şerit polinomlarına göre, görüntüdeki bir y noktasında
        şerit genişliğinin makul (sane) olup olmadığını kontrol eder.
        """
        if left_coeffs is None or right_coeffs is None:
            # İki şerit de bulunamadıysa, genişlik kontrolü yapılamaz.
            # Bu durumu "sane" kabul edip diğer fallback mekanizmalarına bırakıyoruz.
            return True

        # Yorum: Genellikle fit edilen alt bölgenin orta-yüksek bir yerinde width ölçmek daha doğru sonuç verir.
        # offset ve center hesapladığın y_eval_px ile aynı noktayı kullanabilirsin.
        eval_ratio = self.get_parameter('lane_center_eval_ratio').get_parameter_value().double_value
        y_eval = int(h * eval_ratio)

        x_left = self.poly_eval(left_coeffs, y_eval)
        x_right = self.poly_eval(right_coeffs, y_eval)
        
        lane_width = abs(x_right - x_left)
        
        min_width = self.get_parameter('min_lane_width_px').get_parameter_value().integer_value
        max_width = self.get_parameter('max_lane_width_px').get_parameter_value().integer_value

        #self.get_logger().info(f'Genişlik Kontrolü: {lane_width:.0f}px (Beklenen: {min_width}-{max_width}px)')
        if not (min_width < lane_width < max_width):
            self.get_logger().warn(
                f'Sanity Check Başarısız! Genişlik: {lane_width:.0f}px. Beklenen aralık: [{min_width}-{max_width}]px'
            )
            return False
        
        return True

    def visualize_planning(self, mask, left_cluster, right_cluster, lane_center_x, vehicle_center_x, offset, 
                            y_eval_px, left_coeffs=None, right_coeffs=None, x_left_base=None, x_right_base=None, 
                            curvature_left=None, curvature_right=None):
        debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        h, w = mask.shape

        # Yatay hesaplama çizgisi
        cv2.line(debug_img, (0, y_eval_px), (w, y_eval_px), (0, 255, 255), 2) # Sarı yatay çizgi

        # YENİ: HESAPLAMA NOKTALARINI BÜYÜK DAİRELERLE İŞARETLE
        if x_left_base is not None:
            cv2.circle(debug_img, (int(x_left_base), y_eval_px), 12, (255, 255, 0), 3) # Açık Mavi Daire
        if x_right_base is not None:
            cv2.circle(debug_img, (int(x_right_base), y_eval_px), 12, (255, 255, 0), 3) # Açık Mavi Daire

        fit_start = int(h * self.get_parameter('fit_lower_ratio').get_parameter_value().double_value)

        # Sol kenar: yeşil noktalar ve yeşil polinom çizgisi
        if left_cluster is not None:
            for p in left_cluster:
                cv2.circle(debug_img, (p[1], p[0]), 2, (0, 255, 0), -1)
        if left_coeffs is not None and (curvature_left is None or curvature_left < CURVATURE_MAX_THRESHOLD):
            self.draw_polynomial(debug_img, left_coeffs, fit_start, (0, 255, 0), 2)

        # Sağ kenar: kırmızı noktalar ve kırmızı polinom çizgisi
        if right_cluster is not None:
            for p in right_cluster:
                cv2.circle(debug_img, (p[1], p[0]), 2, (0, 0, 255), -1)
        if right_coeffs is not None and (curvature_right is None or curvature_right < CURVATURE_MAX_THRESHOLD):
            self.draw_polynomial(debug_img, right_coeffs, fit_start, (0, 0, 255), 2)

        # Şerit merkezi: mavi çizgi
        cv2.line(debug_img, (int(lane_center_x), h), (int(lane_center_x), int(h - 60)), (255, 0, 0), 3)
        # Araç merkezi: mor çizgi
        cv2.line(debug_img, (int(vehicle_center_x), h), (int(vehicle_center_x), int(h - 60)), (255, 0, 255), 3)
        
        # Bilgi metinleri
        cv2.putText(debug_img, f"Offset: {offset:.2f} px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        if curvature_left is not None and np.isfinite(curvature_left):
            cv2.putText(debug_img, f"L-curv: {curvature_left:.0f}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        if curvature_right is not None and np.isfinite(curvature_right):
            cv2.putText(debug_img, f"R-curv: {curvature_right:.0f}", (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            
        return debug_img

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
