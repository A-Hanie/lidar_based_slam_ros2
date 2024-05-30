#!/usr/bin/env python3


import numpy as np
import os
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pointcloud_interfaces.srv import GetPointCloud2, SetNoiseValues
from ament_index_python.packages import get_package_share_directory

class PointCloudPublisherNode(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        # self.path_point_cloud = 'data/data/'
        
        package_name = 'point_cloud_reader'  
        self.path_point_cloud = os.path.join(get_package_share_directory(package_name), 'data/data')
        
        self.file_names_point_cloud = self.load_file_names()
        self.noise_mean = 0.0
        self.noise_stddev = 0.0

        # Services
        self.srv_get_point_cloud = self.create_service(GetPointCloud2, 'get_point_cloud', self.handle_get_point_cloud_request)
        self.srv_set_noise_values = self.create_service(SetNoiseValues, 'set_noise_values', self.handle_set_noise_values_request)

    def handle_get_point_cloud_request(self, request, response):
        index = request.index
        if index < len(self.file_names_point_cloud):
            self.get_logger().info(f'Reading file at index: {index}')
            point_cloud_msg = self.read_point_cloud_from_file(index)
            response.point_cloud = point_cloud_msg
            self.get_logger().info(f'Sending point cloud with {len(point_cloud_msg.data) // point_cloud_msg.point_step} points.')
            self.get_logger().info(f'Noise parameters: Mean = {self.noise_mean}, StdDev = {self.noise_stddev}')

        else:
            self.get_logger().info('Invalid index: No such file.')
        return response

    def handle_set_noise_values_request(self, request, response):
        self.noise_mean = request.mean
        self.noise_stddev = request.stddev
        self.get_logger().info(f'Updated noise parameters: Mean = {self.noise_mean}, StdDev = {self.noise_stddev}')
        response.success = True
        return response

    def read_point_cloud_from_file(self, index):
        file_path = os.path.join(self.path_point_cloud, self.file_names_point_cloud[index])
        point_list = []
        try:
            with open(file_path, 'rb') as file:
                while True:
                    point_data = file.read(16) 
                    if len(point_data) == 16:
                        point = list(struct.unpack('ffff', point_data))
                        # Add Gaussian noise to x, y, z
                        noise = np.random.normal(self.noise_mean, self.noise_stddev, 3)
                        point[:3] = np.add(point[:3], noise)  # Apply noise
                        point_list.append(tuple(point))
                    else:
                        break
        except IOError as e:
            self.get_logger().info(f'Could not read point cloud file: {file_path}, Error: {e}')
            exit(1)

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = self.get_clock().now().to_msg()
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
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

    def load_file_names(self):
        try:
            return [f for f in os.listdir(self.path_point_cloud) if os.path.isfile(os.path.join(self.path_point_cloud, f))]
        except Exception as e:
            self.get_logger().error(f'Error accessing file path: {e}')
            return []

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisherNode()
    node.get_logger().info("PointCloudPublisherNode is ready and waiting for requests...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
