from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch, generate_move_group_launch, generate_rsp_launch, generate_spawn_controllers_launch
from ament_index_python import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(
            "cobot0", package_name="single_cobot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('single_cobot_moveit_config'),
                'config',
                'cobot0.urdf.xacro'
            )
        )
    ).to_moveit_configs()

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="demo.rviz",
        description="RViz configuration file"
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ar3_bringup"), "config", rviz_base])

    spawn_controllers = generate_spawn_controllers_launch(moveit_config)
    move_group = generate_move_group_launch(moveit_config)
    static_tf = generate_static_virtual_joint_tfs_launch(moveit_config)
    robot_state = generate_rsp_launch(moveit_config)

    static_tf_table = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_transforms_pub",
        output="log",
        arguments=['--x', "0",
                   '--y', "0",
                   '--z', "0",
                   '--yaw', '0',
                   '--pitch', '0',
                   '--roll', '0',
                   '--frame-id', "world",
                   '--child-frame-id', "table_frame"
                   ]
    )

    static_tf_chessboard = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_chessboard_transforms_pub",
        output="log",
        arguments=['--x', "0",
                   '--y', "0",
                   '--z', "0.02877",
                   '--yaw', '-1.570796327',
                   '--pitch', '0',
                   '--roll', '0',
                   '--frame-id', "table_frame",
                   '--child-frame-id', "chessboard_frame"
                   ]
    )

    static_tf_calib_aruco = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_transforms_pub",
        output="log",
        arguments=['--x', "-0.4573",
                   '--y', "-0.1723",
                   '--z', "0",
                   '--yaw', '0',
                   '--pitch', '0',
                   '--roll', '3.14159',
                   '--frame-id', "table_frame",
                   '--child-frame-id', "cobot_calibration_aruco"
                   ]
    )

    ros2_controllers_path = os.path.join(get_package_share_directory(
        "single_cobot_moveit_config"), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ],
        output="both",
    )

    aruco_transforms = Node(
        package='aruco_transforms',
        executable='aruco_transforms',
        name='aruco_transforms',
    )

    cobot_corrector = Node(
        package='cobot_corrector',
        executable='cobot_corrector',
        name='cobot_corrector',
        parameters=[
            {'cobot_prefix': 'cobot0'}
        ]
    )

    tof_piece_finder = Node(
        package='tof_piece_finder',
        executable='tof_piece_finder',
        name='tof_piece_finder'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'rviz2:=WARN'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': False,
            'enable_color': False,
        }],
    )

    chess_controller = Node(
        package='chess_controller',
        executable='chess_controller',
        name="chess_controller"
    )

    chess_player = Node(
        package='chess_player',
        executable='chess_player',
        name="chess_player",
        parameters=[
            {'cobot_ns': 'cobot0'},
            {'cobot_color': 'black'},
            {'frames.chessboard': 'chessboard_frame_dynamic'},
        ]
    )

    piece_identifier = Node(
        package="piece_identifier",
        executable="piece_identifier",
        name="piece_identifier",
    )

    servo_yaml = load_yaml("ar3_bringup", "config/cobot0_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    clock_node = Node(
        package="clock_node",
        executable="clock_node",
    )

    gripper_node = Node(
        package="gripper_node",
        executable="gripper_node",
        name="gripper_node",
    )

    kill_old_bridge = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f ros_ws_bridge; sleep 0.5; true'],
        output='log',
    )

    ros_ws_bridge = Node(
        package="ros_ws_bridge",
        executable="ros_ws_bridge",
        name="ros_ws_bridge",
    )

    # Start ros_ws_bridge only after kill_old_bridge has exited so the port is free.
    start_bridge_after_kill = RegisterEventHandler(
        OnProcessExit(target_action=kill_old_bridge, on_exit=[ros_ws_bridge])
    )

    dashboard = ExecuteProcess(
        cmd=['/home/cobot/cobot-dash/cobot-dash/cobot_dashboard/build/linux/x64/release/bundle/cobot_dashboard'],
        output='log',
    )

    return LaunchDescription([
        rviz_config_arg,
        # cobot_corrector,
        static_tf_table,
        static_tf_calib_aruco,
        static_tf_chessboard,
        ros2_control_node,
        spawn_controllers,
        move_group,
        robot_state,
        static_tf,
        aruco_transforms,
        realsense,
        tof_piece_finder,
        rviz,
        chess_controller,
        chess_player,
        servo_node,
        piece_identifier,
        clock_node,
        gripper_node,
        kill_old_bridge,
        start_bridge_after_kill,
        TimerAction(period=5.0, actions=[dashboard]),
    ])
