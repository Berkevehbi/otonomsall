import rclpy
from rclpy.node import Node
import math

#pip install reeds-shepp
import reeds_shepp  # Yüklediğimiz kütüphaneyi import ediyoruz

# Gerekli Mesaj Tipleri
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info("Path Planner node has been started.")
        
        self.declare_parameter('turning_radius', 4.5)  
        self.turning_radius = self.get_parameter('turning_radius').value

        self.current_pose = None 

        self.target_subscriber = self.create_subscription(
            PoseStamped,
            '/huncar/parking_target_pose',
            self.target_callback,
            10)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10)

        self.path_publisher = self.create_publisher(Path, '/huncar/planned_path', 10)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def target_callback(self, msg: PoseStamped):

        if self.current_pose is None:
            self.get_logger().warn("Current vehicle position is unknown. Cannot plan path yet.")
            return

        self.get_logger().info(f"New target received. Planning path from current pose to target.")

        start_pose = self.current_pose
        end_pose = msg.pose

        self.calculate_and_publish_path(start_pose, end_pose)

    def calculate_and_publish_path(self, start: Pose, end: Pose):

        start_q = [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w]
        _, _, start_yaw = euler_from_quaternion(start_q)
        start_config = (start.position.x, start.position.y, start_yaw)

        end_q = [end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w]
        _, _, end_yaw = euler_from_quaternion(end_q)
        end_config = (end.position.x, end.position.y, end_yaw)

        path_points = reeds_shepp.path_sample(
            start_config, 
            end_config, 
            self.turning_radius, 
            step_size=0.2 
        )
        
        if not path_points:
            self.get_logger().error("No Reeds-Shepp path could be found to the target.")
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom" 
        
        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            x, y, yaw = point
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y

            q = quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Successfully planned and published a path with {len(path_msg.poses)} points.")


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlanner()
    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()