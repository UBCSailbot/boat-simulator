#!/usr/bin/env python3

"""The ROS node for data collection."""

import inspect
import json
import sys
from typing import Type

import custom_interfaces.msg
import rclpy
import rclpy.utilities
import rosbag2_py
import rosidl_runtime_py
from rclpy.node import Node
from rclpy.serialization import serialize_message

import boat_simulator.common.constants as Constants


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    rclpy.spin(node)
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
        self.__init_config_attributes()
        self.__init_storage_files()
        self.__init_msg_types_dict()
        self.__init_subscriptions()
        self.__init_timer_callbacks()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node.
        """
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

    def __init_config_attributes(self):
        """Initialize ROS parameters from the global configuration as private attributes."""
        self.__file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.__record_topics = (
            self.get_parameter("topics").get_parameter_value().string_array_value
        )
        self.__use_bag = self.get_parameter("bag").get_parameter_value().bool_value
        self.__use_json = self.get_parameter("json").get_parameter_value().bool_value

    def __init_storage_files(self):
        if self.__use_bag:
            self.__writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=self.__file_name, storage_id="sqlite3"
            )
            converter_options = rosbag2_py._storage.ConverterOptions("", "")
            self.__writer.open(storage_options, converter_options)

        if self.__use_json:
            self.__data_to_write = {}
            self.__json_counter = 0
            json_file_path = "./" + self.__file_name + ".json"
            self.__json_file = open(json_file_path, "a")
            self.__json_file.write("[\n")

    def __init_msg_types_dict(self):
        """Prepare dictionary of all msg types with key name and value class"""
        self.__msg_types_dict = {}
        for name, cls in inspect.getmembers(custom_interfaces.msg, inspect.isclass):
            if not name.startswith("_"):
                self.__msg_types_dict[name] = cls

    def __init_subscriptions(self):
        """Initialize the subscriptions for this node. These subscriptions pertain to the topics
        from which data will be collected."""
        self.get_logger().debug("Initializing subscriptions...")

        def create_subscription_with_callback(topic_name, msg_type):
            return self.create_subscription(
                msg_type=msg_type,
                topic=topic_name,
                callback=lambda msg: self.__general_sub_callback(msg, topic_name),
                qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            )

        # assuming topics are specified as ['topic_name', 'topic_type']
        for i in range(1, len(self.__record_topics), 2):
            topic_name = self.__record_topics[i - 1]
            msg_type_name = self.__record_topics[i]

            if msg_type_name not in self.__msg_types_dict:
                # Display Error
                continue

            self.__record_topics_sub.append(
                create_subscription_with_callback(topic_name, self.__msg_types_dict[msg_type_name])
            )

            if self.__use_bag:
                topic_info = rosbag2_py._storage.TopicMetadata(
                    name=topic_name, type=msg_type_name, serialization_format="cdr"
                )
                self.__writer.create_topic(topic_info)

            if self.__use_json:
                self.__data_to_write[topic_name] = None

    def __init_timer_callbacks(self):
        """Initializes timer callbacks of this node that are executed periodically."""
        self.get_logger().debug("Initializing timer callbacks...")

        # specify timer period when known
        self.create_timer(timer_period_sec="", callback=self.__write_to_json)

    # SUBSCRIPTION CALLBACKS
    def __general_sub_callback(self, msg: Type, topic_name: str):
        if self.__use_bag:
            self.__writer.write(
                topic_name, serialize_message(msg), self.get_clock().now().nanoseconds
            )
        if self.__use_json:
            msg_as_json = json.dumps(rosidl_runtime_py.message_to_ordereddict(msg), indent=4)
            self.__data_to_write[topic_name] = msg_as_json

    # TIMER CALLBACKS
    def __write_to_json(self):
        # Only concern with this is if topic subscribed to is not launched, then we will never
        # write to json for all the other ones will never get written
        if all(self.__data_to_write.values()):  # all values are not None
            self.__data_to_write["time"] = self.get_clock().now().nanoseconds
            self.__json_counter.write(",\n")  # don't print comma for last
            item_to_write = {self.__json_counter: self.__data_to_write}
            json.dump(item_to_write, self.__json_file, indent=4)
            self.__json_counter += 1

    # Use closures: https://www.geeksforgeeks.org/python-closures/
    # and higher order functions: https://geeksforgeeks.org/higher-order-functions-in-python/
    # Print warning maybe for nodes that haven't published in a while?
    # alternative to checking if node specified in globals.yaml is actually launched
    # Look at the rclpy.Context class for on_shutdown function. Define a shutdown callback
