from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # MTC Demo node
    mtc_spray = Node(
        package="nolon_robot_pkg",
        executable="mtc_spray",
        name="mtc_spray",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([mtc_spray])