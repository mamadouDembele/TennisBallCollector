from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share1 = FindPackageShare('robot_tennis').find('robot_tennis')
    pkg_share2 = FindPackageShare('gazebo_ros').find('gazebo_ros')
    model_file = os.path.join(pkg_share1, 'urdf', "robot_tennis.urdf")
    path = os.path.join(pkg_share2, 'launch', "gazebo.launch.py")
    
    robot_state_publisher_node=Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        arguments=[model_file]
        )
    
    rqt_robot_steering_node=Node(package="rqt_robot_steering", executable="rqt_robot_steering")
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path]),
             )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot_tennis', '-topic', '/robot_description'])

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        rqt_robot_steering_node
    ])