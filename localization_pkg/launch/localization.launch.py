import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    rviz_config_path = launch.substitutions.LaunchConfiguration(
        'rviz_config_path',
        default=os.path.join(
            get_package_share_directory('localization_pkg'),
            'rviz',
            'localization.rviz'))

    # localization 
    localization_node = launch_ros.actions.Node(
        package='localization_pkg',
        executable='localization_node',
        name='localization_node',
        remappings=[('/input_cloud', '/point_cloud')],
        output='screen'
    )
    

    rviz_text = launch_ros.actions.Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_1',
        output='screen',
        parameters=[
            {"string_topic": "rivz_text"},
            {"fg_color": "g"},
        ]
    )
        

    #  RViz 
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        name='rviz2'
    )
    
    # Statistics
    Statistics_node = launch_ros.actions.Node(
        package='ndt_slam',
        executable='Statistics.py',
        arguments=['-d', rviz_config_path],
        name='Statistics_node'
    )   
    
    # PC
    pointcloud_reader_node = launch_ros.actions.Node(
        package='point_cloud_reader',
        executable='pointcloud_node.py',
        name='pointcloud_node'
    )  
    
    

    try:
        return launch.LaunchDescription([
            launch.actions.DeclareLaunchArgument(
                'rviz_config_path',
                default_value=rviz_config_path,
                description='Path to RViz configuration file for visualization'),
            rviz_node,
            localization_node,
            Statistics_node,
            # pointcloud_reader_node,
            rviz_text
        ])
    except Exception as e:
        print(f"Failed to create launch description: {e}")
        raise

