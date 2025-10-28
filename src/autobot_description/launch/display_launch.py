import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("autobot_description")
    default_model_path = os.path.join(pkg_dir, "urdf", "autobot_description.urdf")
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': False}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path),
        robot_state_publisher_node,
        
    ])