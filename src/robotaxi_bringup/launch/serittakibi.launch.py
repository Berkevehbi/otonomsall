# perception_planning_hw.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Kamera tabanlı perception + şerit planlama + PID kontrol + donanım arayüzü.
    LIDAR düğümü YOK.
    """
    return LaunchDescription([
        # 1) Perception (YOLOP tabanlı)
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
                {'kf': 2.5},
                {'target_speed': 0.3},
                {'max_steering_angle': 1.0},
            ]
        ),

        # 3) Davranış Planlayıcı
        Node(
            package='robotaxi_behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner_node',
            output='screen'
        ),

        # 4) Yol Planlayıcı (Path Planner)
        Node(
            package='robotaxi_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                # ROI ve fit oranları
                {'roi_height_ratio': 0.40},   # alt %50 (turuncu çizgi buna göre)
                {'fit_lower_ratio': 0.60},    # ROI’nin yaklaşık alt %80’iyle fit

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

                # Güven düşükken sıçrama kapısı
                {'jump_limit_px': 50},
            ]
        ),

        # 5) Donanım Arayüzü
        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node',
            name='hw_interface_node',
            output='screen'
        ),
    ])
