from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        Node(
            package="track_srv",
            executable="server",
            name="track_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time
            }],
        ),

        Node(
            package="sim_node",
            executable="sim",
            name="sim",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time
            }],
        ),
    ])
