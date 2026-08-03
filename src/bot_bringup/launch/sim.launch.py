import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_description = get_package_share_directory('bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(pkg_description, 'urdf', 'bot.urdf.xacro')
    rviz_config = os.path.join(pkg_bringup, 'config', 'sim.rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the robot simulation configuration',
    )

    # Gazebo rewrites package:// mesh URIs as model:// URIs. Add the parent of
    # the package share directory so model://bot_description resolves without
    # replacing any resource paths provided by ROS or the user's environment.
    gazebo_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_description),
        prepend=True,
    )

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
        parameters=[{
            'robot_description': robot_description_full,
            'use_sim_time': True,
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': True}],
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
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/world/empty/model/my_bot/joint_state@'
                   'sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[
            ('/world/empty/model/my_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )
    return LaunchDescription([
        declare_rviz,
        gazebo_resource_path,
        gazebo,
        rsp,
        rviz,
        spawn_entity,
        bridge
    ])
