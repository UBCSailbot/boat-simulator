#!/usr/bin/env python3

"""The ROS node for data collection."""

import inspect
import sys
from typing import Type

import custom_interfaces.msg
import rclpy
import rclpy.utilities
import rosbag2_py
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
        for name, cls in inspect.getmembers(custom_interfaces.msg, inspect.isclass):
            if not name.startswith("_"):
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

    def __init_subscriptions(self):
        self.get_logger().debug("Initializing subscriptions...")

        topics_to_record = self.get_parameter("topics").get_parameter_value().string_array_value

        # assuming topics are specified as ['topic_name', 'topic_type']
        for i in range(1, len(topics_to_record), 2):
            if topics_to_record[i] in self.__msg_types_dict:
                self.__record_topics_sub.append(
                    self.create_subscription(
                        msg_type=self.__msg_types_dict[topics_to_record[i]],
                        # what happens when try to record topic not running?
                        topic=topics_to_record[i - 1],
                        callback=self.__general_sub_callback,  # to change when implemented
                        qos_profile=self.get_parameter("qos_depth")
                        .get_parameter_value()
                        .integer_value,
                    )
                )
            else:
                # Display Error
                pass

            if self.get_parameter("bag").get_parameter_value().bool_value:
                topic_info = rosbag2_py._storage.TopicMetadata(
                    name=topics_to_record[i - 1], type="", serialization_format="cdr"
                )
                self.__writer.create_topic(topic_info)

    # SUBSCRIPTION CALLBACKS
    def __general_sub_callback(self, msg: Type):
        if self.get_parameter("bag").get_parameter_value().bool_value:
            self.__bag_sub_callback(msg)
        if self.get_parameter("json").get_parameter_value().bool_value:
            self.__json_sub_callback(msg)

    # Use closures: https://www.geeksforgeeks.org/python-closures/
    # and higher order functions: https://geeksforgeeks.org/higher-order-functions-in-python/
    # Print warning maybe for nodes that haven't published in a while?
    # alternative to checking if node specified in globals.yaml is actually launched

    def __bag_sub_callback(self, msg: Type):
        self.__writer.write(
            "topic_name", serialize_message(msg), self.get_clock().now().nanoseconds
        )

    def __json_sub_callback(self, msg: Type):
        pass
