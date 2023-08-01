import os
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

# Global launch arguments and constants. Should be the same across all launch files.
ROS_PACKAGES_DIR = os.path.join(os.getenv("ROS_WORKSPACE"), "src")
GLOBAL_LAUNCH_ARGUMENTS = [
    DeclareLaunchArgument(
        name="config",
        default_value=os.path.join(ROS_PACKAGES_DIR, "global_launch", "config", "globals.yaml"),
        description="Path to ROS parameter config file.",
    ),
    # Reference: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
    DeclareLaunchArgument(
        name="log_level",
        default_value=["info"],
        description="Logging level",
    ),
    DeclareLaunchArgument(
        name="mode",
        default_value="simulation",
        choices=["production", "simulation"],
        description="System mode.",
    ),
]

# Local launch arguments and constants
PACKAGE_NAME = "boat_simulator"
PHYSICS_ENGINE_NODE_NAME = "physics_engine_node"

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
    return LaunchDescription(
        [*GLOBAL_LAUNCH_ARGUMENTS, *LOCAL_LAUNCH_ARGUMENTS, OpaqueFunction(function=setup_launch)]
    )


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
    return launch_description_entities


def get_physics_engine_description(context: LaunchContext) -> Node:
    """Gets the launch description for the physics_engine_node node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_main node.
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
        namespace=PACKAGE_NAME,
        executable=PHYSICS_ENGINE_NODE_NAME,
        name=PHYSICS_ENGINE_NODE_NAME,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node
