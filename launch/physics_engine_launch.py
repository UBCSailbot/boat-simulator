# Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='boat_simulator',
            namespace='boat_simulator',
            executable='physics_engine_node',
            name='physics_engine_node'
        )
    ])
