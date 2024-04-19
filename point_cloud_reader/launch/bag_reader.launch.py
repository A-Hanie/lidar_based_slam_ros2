from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess



def generate_launch_description():
    
    BAG_FILE = 'bags'
    # BAG_FILE = 'rosbag2_2023_02_17-19_02_40'

    pkg_share = FindPackageShare(package='point_cloud_reader')

    bag_file_path = PathJoinSubstitution([
        pkg_share,
        'bags',
        BAG_FILE
    ])

    play_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path],
        output='screen'
    )

    return LaunchDescription([
        play_bag_process
    ])