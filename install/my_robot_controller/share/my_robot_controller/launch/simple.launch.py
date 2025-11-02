from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius" , 
        default_value="0.1"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.45"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mobile_robot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )
    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions = [
            Node(
                package="controller_manager",
                executable = "spawner",
                name="simple_velocity_controller_spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),
            Node(
                package="my_robot_controller",
                executable="simple_controller.py",
                name="simple_controller_py",
                parameters=[{"wheel_radius": wheel_radius,
                             "wheel_separation": wheel_separation ,}]
            )
        ]
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        wheel_controller_spawner,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller
    ])