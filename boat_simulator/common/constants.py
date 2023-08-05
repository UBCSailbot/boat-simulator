"""Constants used across the boat simulator package."""

from dataclasses import dataclass
from enum import Enum


# Class declarations for constants
@dataclass
class PhysicsEngineSubscriptionTopics:
    DESIRED_HEADING: str = "desired_heading"


@dataclass
class PhysicsEnginePublisherTopics:
    GPS: str = "mock_gps"
    KINEMATICS: str = "mock_kinematics"
    WIND_SENSORS: str = "mock_wind_sensors"


# Accessible constants
PHYSICS_ENGINE_PUBLISHERS = PhysicsEnginePublisherTopics()
PHYSICS_ENGINE_SUBSCRIPTIONS = PhysicsEngineSubscriptionTopics()
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"])
QOS_DEPTH = 10
