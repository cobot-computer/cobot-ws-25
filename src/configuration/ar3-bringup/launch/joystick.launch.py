from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Arguments
    serial_port = LaunchConfiguration("port")
    serial_port_launch_arg = DeclareLaunchArgument(
        "port", default_value=TextSubstitution(text="/dev/ttyCobot1")
    )
    baudrate = LaunchConfiguration("baudrate")
    baudrate_launch_arg = DeclareLaunchArgument(
        "baudrate", default_value=TextSubstitution(text="115200")
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("ar3", package_name="ar3_moveit_config").robot_description(
            file_path=os.path.join(
                get_package_share_directory("ar3_moveit_config"),
                "config",
                "ar3.urdf.xacro",
            ),
            mappings={"serial_port": serial_port, "baud_rate": baudrate},
        ).joint_limits(
            file_path=os.path.join(
                get_package_share_directory("ar3_moveit_config"),
                "config",
                "joint_limits.yaml",
            ),
        ).pilz_cartesian_limits(
            file_path=os.path.join(
                get_package_share_directory("ar3_moveit_config"),
                "config",
                "pilz_cartesian_limits.yaml",
            ),
        ).robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("ar3_moveit_config"),
                "config",
                "kinematics.yaml",
            ),
        ).robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("ar3_moveit_config"),
                "config",
                "ar3.srdf",
            ),
        )
    ).to_moveit_configs()

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("ar3_moveit_config", "config/ar3_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ar3_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ar3_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ar3_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as components
    container = ComposableNodeContainer(
        name="ar3_moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[
                    {"child_frame_id": "/ar3_world_joint", "frame_id": "/world"}
                ],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    for f in [
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]:
        print()
        print(f)
        print()

    servo_node = Node(
        package="joystick_servo",
        executable="joystick_servo",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        # arguments=['--ros-args', '--log-level', 'debug'],
        output="screen",
    )

    return LaunchDescription([
        serial_port_launch_arg,
        baudrate_launch_arg,

        # rviz_node,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # ar3_arm_controller_spawner,
        servo_node,
        # container,
    ])
