import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='2d_lidar_detection',
            executable='2d_lidar_detection',
            namespace='lidar_detection',
            output='screen'
            ),
        ])