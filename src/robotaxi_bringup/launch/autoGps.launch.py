from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robotaxi_hw_interface',
            executable='hw_interface_node2',
            name='hw_interface_node2',
            output='screen'
        ),

        Node(
            package='robotaxi_mission_planner',
            executable='mission_planner_node',
            name='mission_planner_node',
            output='screen',
            arguments=['/root/robotaxi_ws/src/robotaxi_mission_planner/robotaxi_mission_planner/birincitur.geojson']  # BURASI EKLENDİ!
        ),
    ])
