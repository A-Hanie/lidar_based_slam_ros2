#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('statistics_node')
        
        # Configuration
        self.number_of_sample = 20
        self.ground_truth_path_file = get_package_share_directory("localization_pkg") + '/maps/final_map_path.csv'
        print(self.ground_truth_path_file)
        
        # Publishers
        self.visualization_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.string_publisher = self.create_publisher(String, 'rivz_text', 10)
        self.ground_truth_publisher = self.create_publisher(Path, 'ground_truth_path', 10)
        
        # Subscribers
        self.subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
        
        # Variables
        self.point_list = []
        self.marker_id = 0
        
        self.timer = self.create_timer(5.0, self.publish_ground_truth_path)

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

    def publish_ground_truth_path(self):
        path = Path()
        path.header.frame_id = 'map'
        try:
            with open(self.ground_truth_path_file, mode='r') as csvfile:
                csv_reader = csv.reader(csvfile, delimiter=',')
                first_line = True
                for row in csv_reader:
                    if first_line:
                        first_line = False
                        continue
                    try:
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.pose.position.x = float(row[1])
                        pose.pose.position.y = float(row[2])
                        pose.pose.position.z = float(row[3])
                        pose.pose.orientation.x = float(row[4])
                        pose.pose.orientation.y = float(row[5])
                        pose.pose.orientation.z = float(row[6])
                        pose.pose.orientation.w = float(row[7])

                        path.poses.append(pose)
                    except ValueError:
                        continue
        except Exception as e:
            self.get_logger().error(f'Failed to read ground truth file: {e}')
        else:
            self.ground_truth_publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
