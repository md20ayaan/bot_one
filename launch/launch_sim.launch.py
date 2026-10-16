import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bot_one'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Fetch the location of your newly compiled custom 3D maze world map asset
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, 'worlds', 'maze_world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': f'-r --render-engine ogre2 {world_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.2'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/my_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/my_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/my_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/my_bot/cmd_vel', '/cmd_vel'),
            ('/model/my_bot/odometry', '/odom'),
            ('/model/my_bot/tf', '/tf')
        ],
        output='screen'
    )

    return LaunchDescription([rsp, gazebo, spawn_entity, bridge])
