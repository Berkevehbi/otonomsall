# autonomous_drive.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Tüm otonom şerit takip sistemini başlatan launch dosyası."""
    
    return LaunchDescription([
        
        # Düğüm 1: Donanım Arayüzü (Motor ve Direksiyon)
        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node',
            name='hw_interface_node',
            output='screen'
        ),
        
        # Düğüm 2: Algılama (Göz - YOLOP)
        Node(
            package='robotaxi_perception',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),

        # Düğüm 3: Yol Planlayıcı (Stratejist)
        Node(
            package='robotaxi_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                {'roi_height_ratio': 0.4},
                {'fit_lower_ratio': 0.6},
                {'dbscan_eps': 40},
                {'dbscan_min_samples': 65},
                {'min_cluster_points': 60},
                {'smoothing_window_size': 5},
                {'default_lane_width_px': 350},
                {'min_lane_width_px': 250},
                {'max_lane_width_px': 1500},
                {'max_points_for_fit': 4000},
                {'lane_center_eval_ratio': 0.75}
            ]
        ),

        # Düğüm 4: PID Kontrolcü (Şoför)
        Node(
            package='robotaxi_control',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[
                # YORUM: Parametreler hıza duyarlı PID için güncellendi.
                # Düşük Hız Ayarları
                {'kp_low_speed': 0.3},
                {'kd_low_speed': 0.05},
                
                # Yüksek Hız Ayarları
                {'kp_high_speed': 0.1},
                {'kd_high_speed': 0.2},
                
                # Hız Eşikleri (m/s)
                {'low_speed_threshold': 2.0},
                {'high_speed_threshold': 10.0},
                
                # Diğer Kontrol Parametreleri
                {'ki': 0.0},
                {'kf': 1.0},
                {'target_speed': 0.3},
                {'max_steering_angle': 1.0}
            ]
        ),     
    ])






"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Donanım Arayüzü
        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node',
            name='hw_interface_node',
            output='screen'
        ),

        # Perception (YOLOP, perception_node)
        Node(
            package='robotaxi_perception',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),

        # Dynamic Obstacle Tracker
        Node(
            package='robotaxi_perception',
            executable='dynamic_obstacle_tracker_node',
            name='dynamic_obstacle_tracker_node',
            output='screen'
        ),

        # Yol Planlayıcı
        Node(
            package='robotaxi_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                {'roi_height_ratio': 0.4},
                {'fit_lower_ratio': 0.6},
                {'dbscan_eps': 40},
                {'dbscan_min_samples': 65},
                {'min_cluster_points': 60},
                {'smoothing_window_size': 5},
                {'default_lane_width_px': 350},
                {'min_lane_width_px': 250},
                {'max_lane_width_px': 1500},
                {'max_points_for_fit': 4000},
                {'lane_center_eval_ratio': 0.75}
            ]
        ),

        # PID Kontrolcü
        Node(
            package='robotaxi_control',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[
                {'kp_low_speed': 0.3},
                {'kd_low_speed': 0.05},
                {'kp_high_speed': 0.1},
                {'kd_high_speed': 0.2},
                {'low_speed_threshold': 2.0},
                {'high_speed_threshold': 10.0},
                {'ki': 0.0},
                {'kf': 0.5},
                {'target_speed': 0.3},
                {'max_steering_angle': 1.0}
            ]
        ),

        # Davranış Planlayıcı (Beyin)
        Node(
            package='robotaxi_behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner_node',
            output='screen'
        ),

        # Localization Node (Konumlama)
        Node(
            package='robotaxi_localization',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),
    ])

"""