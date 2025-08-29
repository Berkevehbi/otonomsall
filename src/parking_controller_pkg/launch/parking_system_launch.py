import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # parking_controller_pkg paketinin yolunu bul
    pkg_share_dir = get_package_share_directory('parking_controller_pkg')
    
    # Parametre dosyasının tam yolunu oluştur
    param_file = os.path.join(pkg_share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        # Beyin ve Kontrol Düğümü
        Node(
            package='parking_controller_pkg',
            executable='parking_controller_node',
            name='parking_controller',
            output='screen',
            parameters=[param_file] # Parametreleri yükle
        ),
        
        # Navigatör / Yörünge Planlayıcı Düğümü
        Node(
            package='parking_controller_pkg',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
            parameters=[param_file] # Parametreleri yükle
        ),

        # Gözler / Çizgi Tespit Düğümü
        Node(
            package='line_detector_pkg',
            executable='line_detector_node',
            name='line_detector',
            output='screen'
        )
    ])