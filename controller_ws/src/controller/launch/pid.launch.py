import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    pkg_share = get_package_share_directory("controller")
    config_file_path = os.path.join(pkg_share, 'config', 'config.yaml')

    controller_node = Node(
        package= 'controller',
        executable= 'longitudinal_controller',
        name = 'longitudinal_controller',
        output = 'screen',
        parameters= [config_file_path]
    )

    return LaunchDescription([
        controller_node
    ])