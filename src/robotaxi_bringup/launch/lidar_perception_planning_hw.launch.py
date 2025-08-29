# robotaxi_full.launch.py
#
# Lidar (dynamic_obstacle_tracker) + Kamera Perception + Davranış Planlayıcı (Beyin)
# + Yol Planlayıcı (şerit takibi parametreleri) + PID Kontrolcü + Donanım Arayüzü
#
# Not: Path Planner parametreleri perception_planning_hw.launch.py (şerit takibi)
# dosyasındaki ayarlarla birebir aynı alınmıştır.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Dinamik Engel Takipçisi (Lidar)
        Node(
            package='robotaxi_perception',
            executable='dynamic_obstacle_tracker_node',
            name='dynamic_obstacle_tracker_node',
            output='screen'
        ),

        # 2) Perception (YOLOP tabanlı)
        Node(
            package='robotaxi_perception',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),

        # 2) PID Kontrolcü (Şoför)
        Node(
            package='robotaxi_control',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[
                # Düşük hız PID
                {'kp_low_speed': 0.24},
                {'kd_low_speed': 0.15},

                # Yüksek hız PID
                {'kp_high_speed': 0.1},
                {'kd_high_speed': 0.2},

                # Hız eşikleri (m/s)
                {'low_speed_threshold': 2.0},
                {'high_speed_threshold': 10.0},

                # Diğer
                {'ki': 0.0},
                {'kf': 0.0},
                {'target_speed': 0.3},
                {'max_steering_angle': 1.0},
            ]
        ),

        # 4) Davranış Planlayıcı (Beyin)
        Node(
            package='robotaxi_behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner_node',
            output='screen'
        ),

        # 5) Yol Planlayıcı (Path Planner) — ŞERİT TAKİBİ PARAMETRELERİ
        Node(
            package='robotaxi_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                # ROI ve fit oranları
                {'roi_height_ratio': 0.40},   # alt %60 kullanılmıyor; yorumda alt %50 yazıyordu, burası 0.40 (üst %60 kırp)
                {'fit_lower_ratio': 0.60},    # ROI’nin alt kısmının ~%40 üstünü kırpmadan fit

                # DBSCAN & cluster
                {'dbscan_eps': 40},
                {'dbscan_min_samples': 65},
                {'min_cluster_points': 60},
                {'max_points_for_fit': 4000},  # performans için pratik sınır

                # Genişlik sınırları ve default
                {'default_lane_width_px': 600},
                {'min_lane_width_px': 250},
                {'max_lane_width_px': 750},

                # y_eval oranı
                {'lane_center_eval_ratio': 0.75},

                # Faz 2: kesikli çizgi güçlendirmeleri
                {'use_morph_close': True},
                {'morph_kernel_w': 3},
                {'morph_kernel_h': 15},
                {'morph_iterations': 1},

                # Anizotropik DBSCAN (y eksenini sıkıştır)
                {'dbscan_y_scale': 0.4},

                # y_eval çevresinde taraf seçimi penceresi
                {'side_select_window_px': 40},
                {'side_select_min_pts': 20},

                # Güven düşükken ani sıçrama kapısı
                {'jump_limit_px': 50},
            ]
        ),

        # 6) Donanım Arayüzü (HW Interface)
        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node',
            name='hw_interface_node',
            output='screen'
        ),
    ])
