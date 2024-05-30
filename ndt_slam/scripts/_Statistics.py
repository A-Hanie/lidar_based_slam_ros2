#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Int32
from ament_index_python.packages import get_package_share_directory

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('statistics_node')
        
        # Configuration
        self.number_of_sample = 20
        self.ground_truth_path_file = get_package_share_directory("localization_pkg") + '/maps/ground_truth_poses.csv'
        self.ground_truth_poses = []
        print(self.ground_truth_path_file)

        # Load the ground truth data
        self.load_ground_truth_path()

        # Publishers
        self.visualization_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.string_publisher = self.create_publisher(String, 'rivz_text', 10)
        self.ground_truth_publisher = self.create_publisher(Path, 'ground_truth_path', 10)
        
        # Subscribers
        self.path_subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.index_subscription = self.create_subscription(Int32, 'pointcloud_indexed_index', self.index_callback, 10)

        # Variables
        self.point_list = []
        self.marker_id = 0
        
    def load_ground_truth_path(self):
        try:
            with open(self.ground_truth_path_file, 'r') as file:
                first_line = True
                for line in file:
                    if first_line:
                        first_line = False
                        continue
                    try:
                        parts = line.strip().split(',')
                        x, y, z, qx, qy, qz, qw = map(float, parts[1:8])  
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.position.z = z
                        pose.pose.orientation.x = qx
                        pose.pose.orientation.y = qy
                        pose.pose.orientation.z = qz
                        pose.pose.orientation.w = qw

                        self.ground_truth_poses.append(pose)
                    except ValueError:
                        self.get_logger().warn(f"Skipping invalid line: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f'Failed to load ground truth file: {e}')


    def index_callback(self, msg):
        if msg.data < len(self.ground_truth_poses):
            path = Path()
            path.header.frame_id = 'map'
            path.poses = self.ground_truth_poses[:msg.data + 1]
            self.ground_truth_publisher.publish(path)

    def path_callback(self, msg):
        for pose in msg.poses:
            self.point_list.append((pose.pose.position.x, pose.pose.position.y))
            if len(self.point_list) == self.number_of_sample:
                avg_x = sum(p[0] for p in self.point_list) / self.number_of_sample
                avg_y = sum(p[1] for p in self.point_list) / self.number_of_sample
                self.publish_marker(avg_x, avg_y)
                self.publish_mean_coordinates(avg_x, avg_y)
                self.point_list = []

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = self.marker_id
        self.marker_id += 1

        self.visualization_publisher.publish(marker)

    def publish_mean_coordinates(self, avg_x, avg_y):
        message = String()
        message.data = f"mean_x = {avg_x:.1f} mean_y = {avg_y:.1f}"
        self.string_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
