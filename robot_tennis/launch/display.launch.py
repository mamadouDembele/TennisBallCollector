from launch import LaunchDescription
from launch_ros.actions import Node
import os
#from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_tennis').find('robot_tennis')
    model_file = os.path.join(pkg_share, 'urdf', "robot_tennis.urdf")
    rviz_config_file = os.path.join(pkg_share, 'config', "display.rviz")
    robot_state_publisher_node=Node(
        package="robot_state_publisher", node_executable="robot_state_publisher",
        arguments=[model_file]
        )

    rviz_node = Node(
        package="rviz2", node_executable="rviz2",
        arguments=["-d", rviz_config_file])

    joint_state_publisher_gui_node=Node(
        package="joint_state_publisher_gui", node_executable="joint_state_publisher_gui")
    return LaunchDescription([robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node
    ])
