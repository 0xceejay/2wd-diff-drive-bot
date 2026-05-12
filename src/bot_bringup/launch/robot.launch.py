import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('bot_bringup')

    # Launch package for tele-operating using a joystick
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'teleop.launch.py')
        )
    )

    # Launch micro-ROS agent
    micro_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'micro_ros.launch.py')
        )
    )

    return LaunchDescription([
        teleop_launch,
        micro_ros_launch
    ])
