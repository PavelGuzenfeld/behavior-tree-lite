"""
PX4 SITL + Behavior Tree Demo Launch File

Prerequisites:
1. PX4-Autopilot built with SITL
2. Micro-XRCE-DDS-Agent running
3. px4_msgs package in workspace

Usage:
    # Terminal 1: Start PX4 SITL
    cd ~/PX4-Autopilot
    make px4_sitl gazebo-classic
    
    # Terminal 2: Start Micro-XRCE-DDS Agent
    MicroXRCEAgent udp4 -p 8888
    
    # Terminal 3: Launch this file
    ros2 launch behavior_tree_lite px4_sitl_demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),

        # Behavior Tree Drone Node
        Node(
            package="behavior_tree_lite",
            executable="px4_drone_node",
            name="px4_drone_bt",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Optional: RViz for visualization
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="screen",
        #     arguments=["-d", PathJoinSubstitution([
        #         FindPackageShare("behavior_tree_lite"),
        #         "rviz",
        #         "px4_drone.rviz"
        #     ])],
        # ),
    ])
