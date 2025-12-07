from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('razor_imu'),
        'config',
        'razor.yaml'
    )

    return LaunchDescription([
        Node(
            package='razor_imu',
            executable='razor_imu_node',
            name='razor_imu_node',
            output='screen',
            parameters=[config_file]
        )
    ])