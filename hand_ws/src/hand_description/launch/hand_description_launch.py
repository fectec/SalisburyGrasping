import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # Find the path to the URDF xacro file and RViz configuration in the hand_description package
    urdf_path = os.path.join(
        FindPackageShare('hand_description').find('hand_description'),
        'urdf',
        'salisbury_hand.urdf.xacro'
    )
    rviz_config_path = os.path.join(
        FindPackageShare('hand_description').find('hand_description'),
        'config',
        'rviz_config.rviz'
    )

    # Process the xacro file to generate the robot_description parameter
    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        # Launch the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # Launch the joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # Launch RViz using the configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])