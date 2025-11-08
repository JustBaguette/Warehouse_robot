# launch/robot_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    robot_name = LaunchConfiguration('robot_name')
    
    # Package directories
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # URDF/XACRO file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_wheeled_robot.urdf.xacro')
    # If using xacro: 
    # urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    
    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    
    # SLAM Toolbox config
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    # Controller config (your existing controller.yaml)
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')

    return LaunchDescription([
        # ==== Launch Arguments ====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for visualization'
        ),
        
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Start SLAM Toolbox for mapping'
        ),
        
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Name of the robot'
        ),

        # ==== 1. Robot State Publisher (Publishes URDF and static TFs) ====
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),

        # ==== 2. Joint State Publisher (if you have movable joints) ====
        # Uncomment if your robot has joints that need publishing
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),

        # ==== 3. Your Custom Odometry Publisher ====
        Node(
            package='minibotslam',  # Change to your package name
            executable='tf_updater',  # Name of your odometry node executable
            name='odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='minibotslam',  # Change to your package name
            executable='robotcontroller',  # Name of your odometry node executable
            name='velocity_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ==== 4. Robot Hardware Controller ====
        # This is your existing controller that accepts /cmd_vel and controls motors
        # Node(
        #     package='your_robot_package',  # Change to your package name
        #     executable='robot_controller',  # Name of your controller node
        #     name='robot_controller',
        #     output='screen',
        #     parameters=[controller_config, {'use_sim_time': use_sim_time}]
        # ),

        # ==== 5. Laser Scanner Driver (LiDAR) ====
        # Example for RPLidar - adjust for your specific LiDAR
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     name='rplidar_node',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',  # Adjust to your LiDAR port
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame',
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard',
        #         'use_sim_time': use_sim_time
        #     }]
        # ),
        
        # For other LiDAR types, replace with appropriate driver:
        # - SICK TiM: package='sick_scan_xd'
        # - Velodyne: package='velodyne_driver'
        # - Hokuyo: package='urg_node'

        # ==== 6. IMU Driver (Optional but recommended) ====
        # Example for common IMU sensors
        # Node(
        #     package='imu_tools',
        #     executable='imu_filter_node',
        #     name='imu_filter',
        #     output='screen',
        #     parameters=[{
        #         'use_mag': False,
        #         'use_sim_time': use_sim_time,
        #         'publish_tf': False,  # Odometry handles the main TF
        #         'world_frame': 'enu'
        #     }]
        # ),

        # ==== 7. Robot Localization (EKF - Sensor Fusion) ====
        # Fuses wheel odometry + IMU for better accuracy
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[
        #         os.path.join(pkg_share, 'config', 'ekf.yaml'),
        #         {'use_sim_time': use_sim_time}
        #     ]
        # ),

        # ==== 8. SLAM Toolbox (Mapping and Localization) ====
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_file,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(use_slam)
        ),

        # ==== 9. RViz2 Visualization ====
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     condition=IfCondition(use_rviz)
        # ),

        # ==== 10. Static Transform Publishers (if needed) ====
        # Example: If your URDF doesn't define laser_frame position
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        
        
        # Add more static transforms as needed for sensors
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_imu',
        #     arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_frame']
        # ),
    ])
