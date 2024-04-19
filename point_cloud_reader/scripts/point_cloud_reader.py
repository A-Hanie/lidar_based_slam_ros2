#!/usr/bin/env python3

import os
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data

class pointcloud_Pub_Node(Node):
    def __init__(self):
        super().__init__('pointCloud_publisher')
        self.file_index = 0
        self.time = 0.5 # sec
        self.publisher_point_cloud = self.create_publisher(PointCloud2, '/point_cloud', qos_profile_sensor_data)
        self.path_point_cloud = 'data/data/'
        self.file_names_point_cloud = []
        self.create_publishers_data_file_names()
        self.timer = self.create_timer(self.time, self.on_timer_callback)

    def on_timer_callback(self):
        if self.file_index < len(self.file_names_point_cloud):
            self.get_logger().info(f'Reading file at index: {self.file_index}')
            point_cloud_msg = self.read_point_cloud_from_file()
            self.publisher_point_cloud.publish(point_cloud_msg)
            self.file_index += 1
        else:
            self.get_logger().info('Finished processing all point cloud files.')
            rclpy.shutdown()

    def read_point_cloud_from_file(self):
        file_path = os.path.join(self.path_point_cloud, self.file_names_point_cloud[self.file_index])

        point_list = []
        try:
            with open(file_path, 'rb') as file:
                while True:
                    point_data = file.read(16) 
                    if len(point_data) == 16:
                        point = struct.unpack('ffff', point_data)
                        point_list.append(point)
                    else:
                        break
        except IOError as e:
            self.get_logger().info(f'Could not read point cloud file: {file_path}, Error: {e}')
            exit(1)

        #  PointCloud2 message
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = self.get_clock().now().to_msg()

        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(['x', 'y', 'z', 'intensity'])]

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(point_list)
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 16 
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = np.asarray(point_list, dtype=np.float32).tobytes()

        return point_cloud_msg

    def create_publishers_data_file_names(self):
        try:
            file_names = [entry for entry in os.listdir(self.path_point_cloud) if os.path.isfile(os.path.join(self.path_point_cloud, entry))]
            file_names.sort()
            self.file_names_point_cloud = file_names
        except Exception as e:
            self.get_logger().error(f'File path not found: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = pointcloud_Pub_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
