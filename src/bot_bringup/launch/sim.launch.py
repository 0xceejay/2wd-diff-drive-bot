import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_description = get_package_share_directory('bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(pkg_description, 'urdf', 'bot.urdf.xacro')

    # Convert xacro to urdf
    robot_description_full = Command(['xacro ', xacro_file])

    #  Gazebo Sim Launch
    # '-r' starts the simulation immediately, 'empty.sdf' is the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_full}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
        output='screen'
    )

    # Bridge: Connects ROS 2 topics to Gazebo Sim topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        bridge
    ])
