import importlib
import os
from types import ModuleType
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def get_global_main_launch_module() -> ModuleType:
    """Execute and return the main launch file of the `global_launch` package.

    Returns:
        ModuleType: The executed module.
    """
    global_main_launch = os.path.join(
        os.getenv("ROS_WORKSPACE"), "src", "global_launch", "main_launch.py"
    )
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GLOBAL_LAUNCH = get_global_main_launch_module()


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the `local_pathfinding` package.

    Returns:
        LaunchDescription: The launch description.
    """
    # get_nodes() arguments for the package launch
    common_parameters = [GLOBAL_LAUNCH.GLOBAL_CONFIG]
    common_ros_arguments = [*GLOBAL_LAUNCH.get_log_ros_arguments()]
    mode = LaunchConfiguration("mode")

    return LaunchDescription(
        [
            *GLOBAL_LAUNCH.GLOBAL_LAUNCH_ARGUMENTS,
            *get_nodes(common_parameters, common_ros_arguments, mode),
        ]
    )


def get_nodes(common_parameters: List, common_ros_arguments: List, mode: str) -> List[Node]:
    """Get the nodes to be launched depending on an indicated mode.

    Args:
        common_parameters (List): Parameters that are common to all nodes.
        common_ros_arguments (List): ROS arguments that are common to all nodes.
        mode (str): The system mode.

    Returns:
        List[Node]: The nodes to be launched.
    """
    # physics_engine_node parameters and ROS arguments
    physics_engine_node_parameters = [*common_parameters]
    physics_engine_node_ros_arguments = [*common_ros_arguments]

    nodes = []

    if mode == "simulation":
        nodes.extend(
            [
                Node(
                    package="boat_simulator",
                    namespace="boat_simulator",
                    executable="physics_engine_node",
                    name="physics_engine_node",
                    parameters=physics_engine_node_parameters,
                    ros_arguments=physics_engine_node_ros_arguments,
                ),
            ]
        )

    return nodes
