"""Launch file that runs all nodes for the boat simulator ROS package."""

import importlib
import os
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

# Local launch arguments and constants
PACKAGE_NAME = "boat_simulator"
PHYSICS_ENGINE_NODE_NAME = "physics_engine_node"
LOW_LEVEL_CONTROL_NODE_NAME = "low_level_control_node"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS = [
    DeclareLaunchArgument(
        name="test_sim_argument", default_value="Hello", description="This is a test argument."
    )
]


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `boat_simulator`
    package.

    Returns:
        LaunchDescription: The launch description.
    """
    global_launch_arguments = get_global_launch_arguments()
    local_launch_arguments = LOCAL_LAUNCH_ARGUMENTS
    return LaunchDescription(
        [*global_launch_arguments, *local_launch_arguments, OpaqueFunction(function=setup_launch)]
    )


def get_global_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Gets the global launch arguments defined in the global launch file.

    Returns:
        List[LaunchDescriptionEntity]: List of global launch argument objects.
    """
    global_main_launch = os.path.join(
        os.getenv("ROS_WORKSPACE"), "src", "global_launch", "main_launch.py"
    )
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    return global_launch_arguments


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
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"{PHYSICS_ENGINE_NODE_NAME}:=", LaunchConfiguration("log_level")],
    ]

    # TODO Delete later
    print(LaunchConfiguration("test_sim_argument").perform(context))

    node = Node(
        package=PACKAGE_NAME,
        executable=PHYSICS_ENGINE_NODE_NAME,
        name=PHYSICS_ENGINE_NODE_NAME,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_low_level_control_description(context: LaunchContext) -> Node:
    """Gets the launch description for the low level control node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the low level control node.
    """
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"{LOW_LEVEL_CONTROL_NODE_NAME}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=PACKAGE_NAME,
        executable=LOW_LEVEL_CONTROL_NODE_NAME,
        name=LOW_LEVEL_CONTROL_NODE_NAME,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node
