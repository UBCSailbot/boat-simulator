"""Launch file that runs all nodes for the boat simulator ROS package."""

import importlib
import os
from typing import List

from launch_ros.actions import Node

import boat_simulator.common.constants as Constants
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration

# Local launch arguments and constants
PACKAGE_NAME = "boat_simulator"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="enable_sim_multithreading",
        default_value="false",
        choices=["true", "false"],
        description="Enable multithreaded execution of callbacks in the boat simulator",
    )
]


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `boat_simulator`
    package.

    Returns:
        LaunchDescription: The launch description.
    """
    global_launch_arguments, global_environment_vars = get_global_launch_arguments()
    return LaunchDescription(
        [
            *global_launch_arguments,
            *global_environment_vars,
            *LOCAL_LAUNCH_ARGUMENTS,
            OpaqueFunction(function=setup_launch),
        ]
    )


def get_global_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Gets the global launch arguments defined in the global launch file.

    Returns:
        List[LaunchDescriptionEntity]: List of global launch argument objects.
    """
    ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
    global_main_launch = os.path.join(ros_workspace, "src", "global_launch", "main_launch.py")
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    if spec is None:
        raise ImportError(f"Couldn't import global_launch module from {global_main_launch}")
    module = importlib.util.module_from_spec(spec)  # type: ignore[arg-type] # spec is not None
    spec.loader.exec_module(module)  # type: ignore[union-attr] # spec is not None
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    global_environment_vars = module.ENVIRONMENT_VARIABLES
    return global_launch_arguments, global_environment_vars  # type: ignore[return-value] # no type


def setup_launch(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Collects launch descriptions that describe the system behavior in the `boat_simulator`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[LaunchDescriptionEntity]: Launch descriptions.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_physics_engine_description(context))
    launch_description_entities.append(get_low_level_control_description(context))
    return launch_description_entities


def get_physics_engine_description(context: LaunchContext) -> Node:
    """Gets the launch description for the physics engine node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the physics engine node.
    """
    node_name = "physics_engine_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    local_arguments = [
        Constants.MULTITHREADING_CLI_ARG_NAME,
        [LaunchConfiguration("enable_sim_multithreading")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node


def get_low_level_control_description(context: LaunchContext) -> Node:
    """Gets the launch description for the low level control node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the low level control node.
    """
    node_name = "low_level_control_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    local_arguments = [
        Constants.MULTITHREADING_CLI_ARG_NAME,
        [LaunchConfiguration("enable_sim_multithreading")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node
