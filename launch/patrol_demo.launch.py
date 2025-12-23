from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("behavior_tree_lite")

    urdf_path = os.path.join(pkg_share, "urdf", "minibot.urdf")
    rviz_path = os.path.join(pkg_share, "rviz", "patrol_demo.rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_path, "r").read()}],
    )

    patrol_robot = Node(
        package="behavior_tree_lite",
        executable="patrol_robot_node",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            patrol_robot,
            rviz,
        ]
    )
