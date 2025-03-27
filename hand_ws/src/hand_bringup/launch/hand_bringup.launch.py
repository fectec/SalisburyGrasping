import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directories
    hand_description_share = get_package_share_directory('hand_description')
    hand_algorithms_share = get_package_share_directory('hand_algorithms')
    
    # Include the hand_description launch file (which sets up robot_state_publisher, joint_state_publisher, and RViz)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_description_share, 'launch', 'hand_description.launch.py')
        )
    )
    
    # Launch the grasping node from hand_algorithms
    grasping_node = Node(
        package='hand_algorithms',
        executable='grasping',  
        name='grasping_node',
        output='screen'
    )
    
    return LaunchDescription([
        description_launch,
        grasping_node,
    ])