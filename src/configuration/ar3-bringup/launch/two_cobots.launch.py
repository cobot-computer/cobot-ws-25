from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("two_cobots", package_name="two_cobots_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('two_cobots_moveit_config'),
                'config',
                'two_cobots.urdf.xacro'
            )
        )
    ).to_moveit_configs()

    # Create demo launch description
    demo_desc = generate_demo_launch(moveit_config)

    return LaunchDescription([
        demo_desc
    ])
