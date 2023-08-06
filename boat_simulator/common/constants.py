"""Constants used across the boat simulator package."""

from dataclasses import dataclass
from enum import Enum


# Class declarations for constants
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


# Accessible constants
ACTION_NAMES = Actions()
ACTION_SEND_GOAL_TIMEOUT_SEC = 2

INFO_LOG_THROTTLE_PERIOD_SEC = 2

LOW_LEVEL_CTRL_SUBSCRIPTIONS = LowLevelControlSubscriptionTopics()

MULTITHREADING_CLI_ARG_NAME = "--enable-multithreading"

PHYSICS_ENGINE_PUBLISHERS = PhysicsEnginePublisherTopics()
PHYSICS_ENGINE_SUBSCRIPTIONS = PhysicsEngineSubscriptionTopics()

ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"])

SAIL_ACTUATION_REQUEST_PERIOD_SEC = 10  # TODO Make this a ROS parameter
SAIL_ACTUATION_EXECUTION_PERIOD_SEC = 0.5  # TODO Make this a ROS parameter
SAIL_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated

RUDDER_ACTUATION_REQUEST_PERIOD_SEC = 10  # TODO Make this a ROS parameter
RUDDER_ACTUATION_EXECUTION_PERIOD_SEC = 0.5  # TODO Make this a ROS parameter
RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated

QOS_DEPTH = 1
