# launch/nav2_navigation.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_robot = FindPackageShare('your_robot_package').find('your_robot_package')
    nav2_params_file = os.path.join(pkg_robot, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(pkg_robot, 'maps', 'your_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        # Your odometry publisher node is needed
        Node(
            package='your_robot_package',
            executable='odom_publisher_4wd',
            name='odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
