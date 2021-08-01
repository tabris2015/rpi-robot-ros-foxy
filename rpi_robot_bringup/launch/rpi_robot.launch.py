import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    pico_port = LaunchConfiguration('pico_port', default='/dev/ttyACM0')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/ttyUSB0')


    return LaunchDescription([
        DeclareLaunchArgument(
            'pico_port',
            default_value=pico_port,
            description='Serial port for communication with microcontroller'
        ),
        DeclareLaunchArgument(
            'rplidar_port',
            default_value=rplidar_port,
            description='Port for rplidar sensor'
        ),
        # lidar launchfile
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/lidar.launch.py']
            ),
            launch_arguments={'serial_port': rplidar_port}.items()
        ),
        # robot control node
        Node(
            package='rpi_robot_bringup',
            executable='rpi_robot_control.py',
            parameters=[{
                'pico_port': pico_port
            }],
            arguments=[],
            output='screen'
        ),
        # static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.064', '0', '0.120', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )
    ])