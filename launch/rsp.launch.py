import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Check if we should use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file using Xacro
    pkg_path = get_package_share_directory('bot_one')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    
    # Securely wrap the xacro command output into a ParameterValue string block
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Create the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher
    ])
