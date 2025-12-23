from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('behavior_tree_lite')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'patrol.rviz')
    
    return LaunchDescription([
        # Behavior tree node
        Node(
            package='behavior_tree_lite',
            executable='patrol_robot_node',
            name='patrol_robot',
            output='screen',
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen',
        ),
    ])