import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
import launch_ros



def generate_launch_description():

    pkg_dir = get_package_share_directory('autobot_bringup')
    
    config_dir = os.path.join(get_package_share_directory('autobot_bringup'), 'config')
    
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    autobot_description_launch_path = os.path.join(get_package_share_directory('autobot_description'), 'launch', 'display_launch.py')
    
    autobot_description_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            autobot_description_launch_path
        )
    )
    
    autobot_bringup_node = Node(
        package='autobot_bringup',
        executable='bringup',
        name='bringup',
        output='screen',
    )
    
    autobot_imu_publisher_node = Node(
        package='autobot_bringup',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )
    
    autobot_imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'imu_filter.yaml')],
        remappings=[
            ('/imu/data', '/imu')
        ]
    )
    
    sllidar_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch','sllidar_c1_launch.py')
        ),
    )
    
    autobot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'ekf.yaml'), {'use_sim_time': False}]
    )

    return launch.LaunchDescription(
        [
            autobot_description_launch,
            autobot_bringup_node,
            autobot_imu_publisher_node,
            autobot_imu_filter_node,
            sllidar_launch,
            autobot_localization_node,
        ]
    )