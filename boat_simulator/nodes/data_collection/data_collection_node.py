#!/usr/bin/env python3

"""The ROS node for data collection."""

import inspect
import sys

import custom_interfaces.msg as msg_types
import rclpy
import rclpy.utilities
import rosbag2_py
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import boat_simulator.common.constants as Constants


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


def is_collection_enabled():
    try:
        is_data_collection_enabled_index = (
            sys.argv.index(Constants.DATA_COLLECTION_CLI_ARG_NAME) + 1
        )
        is_data_collection_enabled = sys.argv[is_data_collection_enabled_index] == "true"
    except ValueError:
        is_data_collection_enabled = False
    return is_data_collection_enabled


class DataCollectionNode(Node):
    def __init__(self):
        super.__init__("data_collection_node")
        self.get_logger().debug("Initializing node...")
        self.__declare_ros_parameters()
        self.__init_private_attributes()
        # add rest of init helpers
        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node.
        """
        # assume topis will be an array where it alternates topic then topic type
        self.get_logger().debug("Declaring ROS parameters...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("file_name", rclpy.Parameter.Type.STRING),
                ("topics", rclpy.Parameter.Type.STRING_ARRAY),
                ("bag", rclpy.Parameter.Type.BOOL),
                ("json", rclpy.Parameter.Type.BOOL),
            ],
        ),
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_msg_types_dict(self) -> None:
        """Prepare dictionary of all msg types with key name and value class"""
        self.__msg_types_dict = {}
        for name, cls in inspect.getmembers(msg_types, inspect.isclass):
            self.__msg_types_dict[name] = cls

    def __init_private_attributes(self):
        if self.get_parameter("bag").get_parameter_value().bool_value:
            self.__writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri="data_collection_bag", storage_id="sqlite3"
            )
            converter_options = rosbag2_py._storage.ConverterOptions("", "")
            self.__writer.open(storage_options, converter_options)
        if self.get_parameter("json").get_parameter_value().bool_value:
            # add json init
            pass

    def __init_callback_groups(self):
        """Initializes the callback groups."""
        self.get_logger().debug("Initializing callback groups...")
        # add here when implemented

    def __init_subscriptions(self):
        self.get_logger().debug("Initializing subscriptions...")
        classe_names = [name for name, cls in inspect.getmembers(msg_types, inspect.isclass)]
        for i in range(len(self.get_parameter("topics").get_parameter_value().string_array_value)):
            pass
        self.__desired_heading_sub = self.create_subscription(
            msg_type=DesiredHeading,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.DESIRED_HEADING,
            callback=self.__desired_heading_sub_callback,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            callback_group=self.sub_callback_group,
        )
        topics_to_record = []
        for topic in topics_to_record:
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic, type="", serialization_format="cdr"
            )
            self.writer.create_topic(topic_info)
