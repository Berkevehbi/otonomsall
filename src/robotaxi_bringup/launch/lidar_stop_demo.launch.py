# lidar_stop_demo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Sadece Lidar engel tespiti → davranış planlayıcı → donanım zincirini çalıştırır.
    (minimum çalışma için)
    """
    return LaunchDescription([
        # 1. Dinamik Engel Takipçisi (Lidar)
        Node(
            package='robotaxi_perception',
            executable='dynamic_obstacle_tracker_node',
            name='dynamic_obstacle_tracker_node',
            output='screen'
        ),

        # 2. Davranış Planlayıcı (Beyin)
        Node(
            package='robotaxi_behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner_node',
            output='screen'
        ),

        # 3. Donanım Arayüzü (HW Interface)
        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node',
            name='hw_interface_node',
            output='screen'
        ),
    ])
