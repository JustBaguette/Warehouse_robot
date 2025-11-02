# launch/rplidar.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLidar (e.g., /dev/ttyUSB0)'
        ),
        
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Baudrate for RPLidar communication'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='Frame ID for laser scanner'
        ),
        
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Invert scan data'
        ),
        
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Enable angle compensation'
        ),
        
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard',
            description='Scan mode (Standard, Express, Boost, Sensitivity, Stability)'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # RPLidar node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': LaunchConfiguration('inverted'),
                'angle_compensate': LaunchConfiguration('angle_compensate'),
                'scan_mode': LaunchConfiguration('scan_mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
