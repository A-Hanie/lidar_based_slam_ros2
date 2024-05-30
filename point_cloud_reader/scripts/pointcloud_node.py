#!/usr/bin/env python3

import numpy as np
import os
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from pointcloud_interfaces.srv import SetNoiseValues
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import re
from slam_msgs.msg import PointCloudIndex 


from std_msgs.msg import Int32

def natural_sort_key(s, _nsre=re.compile('([0-9]+)')):
    return [int(text) if text.isdigit() else text.lower() for text in re.split(_nsre, s)]



class PointCloudPublisherNode(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        package_name = 'point_cloud_reader'
        self.path_point_cloud = os.path.join(get_package_share_directory(package_name), 'data')
        self.file_names_point_cloud = self.load_file_names()
        self.noise_mean = 0.0
        self.noise_stddev = 0.0
        self.is_paused = True
        self.current_index = 0

        custom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=500 
        )
        
        # Publishers
        self.publisher = self.create_publisher(PointCloudIndex, 'pointcloud_indexed', custom_qos)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'pointcloud_stream', custom_qos)


        # Services
        self.srv_pause = self.create_service(Trigger, 'pause_slam', self.handle_pause_request)
        self.srv_resume = self.create_service(Trigger, 'resume_slam', self.handle_resume_request)
        self.srv_restart = self.create_service(Trigger, 'reset_pointcloud_index', self.handle_restart_request)
        self.srv_set_noise_values = self.create_service(SetNoiseValues, 'set_noise_values', self.handle_set_noise_values_request)

        # Timer
        self.timer = self.create_timer(0.5, self.publish_point_cloud)

    def publish_point_cloud(self):
        if not self.is_paused and self.current_index < len(self.file_names_point_cloud):
            file_name = self.file_names_point_cloud[self.current_index]
            print(f"File name : {file_name}")
            pc_msg = self.read_point_cloud_from_file(self.current_index)
            index_and_cloud = PointCloudIndex()
            index_and_cloud.index.data = self.current_index
            index_and_cloud.point_cloud = pc_msg
            self.publisher.publish(index_and_cloud)
            self.point_cloud_publisher.publish(pc_msg)
            self.get_logger().info(f'Published point cloud with index: {self.current_index}, with μ: {self.noise_mean}, σ: {self.noise_stddev}')
            self.current_index += 1

    def handle_pause_request(self, request, response):
        self.is_paused = True
        response.success = True
        response.message = 'PointCloud streaming paused'
        return response

    def handle_resume_request(self, request, response):
        self.is_paused = False
        response.success = True
        response.message = 'PointCloud streaming resumed'
        return response

    def handle_restart_request(self, request, response):
        self.current_index = 0
        self.is_paused = True
        response.success = True
        response.message = 'PointCloud streaming restarted from index 0'
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
            file_list = [f for f in os.listdir(self.path_point_cloud) if os.path.isfile(os.path.join(self.path_point_cloud, f))]
            file_list.sort(key=natural_sort_key)  # Sort the files naturally; e.g., 1, 2, 10 instead of 1, 10, 2
            return file_list
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
