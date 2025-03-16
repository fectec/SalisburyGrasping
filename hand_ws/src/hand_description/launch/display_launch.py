import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Find the path to the URDF xacro file
    urdf_path = FindPackageShare('hand_description').find('hand_description') + '/urdf/salisbury_hand.urdf.xacro'
    rviz_config_path = FindPackageShare('hand_description').find('hand_description') + '/config/rviz_config.rviz'

    # Define the robot_description parameter
    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        # Set the robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Start the joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz with a given config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])