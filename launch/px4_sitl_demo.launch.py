from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),

        Node(
            package="behavior_tree_lite",
            executable="px4_vehicle_node",
            name="px4_vehicle_bt",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

    ])
