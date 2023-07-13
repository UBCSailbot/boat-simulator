# Reference: https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:
    physics_engine_launch_include = generate_physics_engine_launch_include()
    launch_description = LaunchDescription([physics_engine_launch_include])
    return launch_description

def generate_physics_engine_launch_include() -> IncludeLaunchDescription:
    launch_file_path = os.path.join(
        get_package_share_directory('boat_simulator'),
        'physics_engine_launch.py'
    )
    launch_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))
    return launch_include
