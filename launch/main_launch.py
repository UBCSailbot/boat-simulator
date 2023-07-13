# Source: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
from launch import LaunchDescription
from launch.actions import Node


def generate_launch_description() -> LaunchDescription:
    # TODO: Add global config

    physics_engine_node = Node(
            package='boat_simulator',
            namespace='boat_simulator',
            executable='physics_engine_node',
            name='physics_engine_node'
        )
    launch_description = LaunchDescription([physics_engine_node])
    return launch_description
