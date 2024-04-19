import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    rviz_config_path = launch.substitutions.LaunchConfiguration(
        'rviz_config_path',
        default=os.path.join(
            get_package_share_directory('ndt_slam'),
            'rviz',
            'mapping.rviz'))

    # NDT SLAM 
    mapping_node = launch_ros.actions.Node(
        package='ndt_slam',
        executable='ndt_slam_node',
        name='ndt_slam_node',
        remappings=[('/input_cloud', '/point_cloud')],
        output='screen'
    )

    #  RViz 
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        name='rviz2'
    )

    try:
        return launch.LaunchDescription([
            launch.actions.DeclareLaunchArgument(
                'rviz_config_path',
                default_value=rviz_config_path,
                description='Path to RViz configuration file for visualization'),
            mapping_node,
            rviz_node,
        ])
    except Exception as e:
        print(f"Failed to create launch description: {e}")
        raise

