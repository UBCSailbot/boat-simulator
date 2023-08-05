"""Constants used across the boat simulator package."""

from dataclasses import dataclass
from enum import Enum


# Class declarations for constants
@dataclass
class ActionClients:
    RUDDER_ACTUATION: str = "rudder_actuation_client"
    SAIL_ACTUATION: str = "sail_actuation_client"


@dataclass
class PhysicsEngineSubscriptionTopics:
    DESIRED_HEADING: str = "desired_heading"


@dataclass
class PhysicsEnginePublisherTopics:
    GPS: str = "mock_gps"
    KINEMATICS: str = "mock_kinematics"
    WIND_SENSORS: str = "mock_wind_sensors"


# Accessible constants
ACTION_CLIENTS = ActionClients()
ACTION_SEND_GOAL_TIMEOUT_SEC = 1
PHYSICS_ENGINE_PUBLISHERS = PhysicsEnginePublisherTopics()
PHYSICS_ENGINE_SUBSCRIPTIONS = PhysicsEngineSubscriptionTopics()
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"])
RUDDER_ACTUATION_REQUEST_PERIOD_SEC = 10  # TODO Make this a ROS parameter
QOS_DEPTH = 10
