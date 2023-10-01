"""Constants used across the boat simulator package."""

from dataclasses import dataclass
from enum import Enum


# Class declarations for constants. These are not meant to be accessed directly.
@dataclass
class Actions:
    RUDDER_ACTUATION: str = "rudder_actuation"
    SAIL_ACTUATION: str = "sail_trim_tab_actuation"


@dataclass
class LowLevelControlSubscriptionTopics:
    GPS: str = "mock_gps"


@dataclass
class PhysicsEngineSubscriptionTopics:
    DESIRED_HEADING: str = "desired_heading"


@dataclass
class PhysicsEnginePublisherTopics:
    GPS: str = "mock_gps"
    KINEMATICS: str = "mock_kinematics"
    WIND_SENSORS: str = "mock_wind_sensors"


# Directly accessible constants

# Boat simulator ROS action names
ACTION_NAMES = Actions()

# How long the action clients wait for the server before timing out
ACTION_SEND_GOAL_TIMEOUT_SEC = 2

# Limits the info logs to avoid overwhelming the terminal ( it's really more for us humans :-) )
INFO_LOG_THROTTLE_PERIOD_SEC = 2

# ROS topic names for the low level control node subscriptions
LOW_LEVEL_CTRL_SUBSCRIPTIONS = LowLevelControlSubscriptionTopics()

# CLI argument name for multithreading option for physics engine
MULTITHREADING_CLI_ARG_NAME = "--enable-multithreading"

# ROS topic names for physics engine publishers
PHYSICS_ENGINE_PUBLISHERS = PhysicsEnginePublisherTopics()

# ROS topic names for physics engine subscriptions
PHYSICS_ENGINE_SUBSCRIPTIONS = PhysicsEngineSubscriptionTopics()

# Enumerated orientation indices since indexing pitch, roll, and yaw could be arbitrary
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"], start=0)  # x, y, x

# How often the sail action client requests a sail actuation
SAIL_ACTUATION_REQUEST_PERIOD_SEC = 10  # TODO Make this a ROS parameter

# How often the sail action server routine's main loop executes
SAIL_ACTUATION_EXECUTION_PERIOD_SEC = 0.5  # TODO Make this a ROS parameter, or same as pub period?

# Number of times the sail action server routine's main loop executes
SAIL_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the ctrl is integrated

# How often the rudder action client requests a rudder actuation
RUDDER_ACTUATION_REQUEST_PERIOD_SEC = 10  # TODO Make this a ROS parameter

# How often the rudder action server routine's main loop executes
RUDDER_ACTUATION_EXECUTION_PERIOD_SEC = 0.5  # TODO Make this a ROS parameter, or same as pub prd.?

# Number of times the rudder action server routine's main loop executes
RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated

# The maximum number of subscription messages to queue for further processing
QOS_DEPTH = 1
