from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Package paths ---
    pkg_desc = get_package_share_directory("my_robot_description")

    # Path to main xacro file (must include ros2_control and gazebo plugin)
    xacro_file = os.path.join(pkg_desc, "urdf", "my_robot.urdf.xacro")

    # --- Environment variables ---
    model_path = os.path.join(pkg_desc, "models")
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=model_path
    )

    # --- Robot Description Parameter ---
    robot_description = Command(["xacro ", xacro_file])

    # --- Launch robot_state_publisher ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # --- Launch Gazebo Classic ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    # --- Spawn the robot into Gazebo ---
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_robot",
            "-x", "0", "-y", "0", "-z", "0.1"
        ],
        output="screen"
    )

    # --- Spawn Controllers (after a short delay to ensure robot is spawned) ---
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # seconds delay
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen"
            )
        ]
    )

    mobile_robot_controller_spawner = TimerAction(
        period=5.0,  # slightly longer delay to ensure joint_state_broadcaster is active
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mobile_robot_controller"],
                output="screen"
            )
        ]
    )

    # --- Final LaunchDescription ---
    return LaunchDescription([
        set_gazebo_model_path,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        joint_state_broadcaster_spawner,
        mobile_robot_controller_spawner,
    ])
