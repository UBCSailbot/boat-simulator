#!/usr/bin/env python3

"""The ROS node for data collection."""
import inspect
import json
import os
import signal
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


def shutdown_handler(signum, frame):
    """Necessary for ros shutdown callback to be properly called."""
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    if is_collection_enabled():
        try:
            signal.signal(signal.SIGINT, shutdown_handler)
            rclpy.spin(node)
        finally:
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
        super().__init__(node_name="data_collection_node")
        self.get_logger().debug("Initializing node...")
        self.__declare_ros_parameters()
        self.__init_storage_files()
        self.__init_msg_types_dict()
        self.__init_subscriptions()
        self.__init_timer_callbacks()
        self.__init_shutdown_callbacks()
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
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("topics", rclpy.Parameter.Type.STRING_ARRAY),
                ("bag", rclpy.Parameter.Type.BOOL),
                ("json", rclpy.Parameter.Type.BOOL),
                ("write_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        ),
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_storage_files(self):
        """Initializes storage files for data logging."""
        self.get_logger().debug("Initializing storrage files...")

        if self.use_json:
            self.__data_to_write = {}
            self.__json_index_counter = 0
            self.__start_time = self.get_clock().now().nanoseconds
            json_file_path = "./" + self.file_name + ".json"
            if os.path.exists(json_file_path):
                self.get_logger().warn(
                    f"JSON file with name {self.file_name} already exists. Overriding old file..."
                )
                os.remove(json_file_path)
            self.__json_file = open(json_file_path, "a")
            self.__json_file.write("[\n")  # Open JSON array

        if self.use_bag:
            self.__writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=self.file_name, storage_id="sqlite3"
            )
            converter_options = rosbag2_py._storage.ConverterOptions("", "")
            self.__writer.open(storage_options, converter_options)

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

        self.__record_topics_sub = []

        # Assuming topics are specified as ['topic_name', 'topic_type']
        for i in range(1, len(self.topics_to_record), 2):
            topic_name = self.topics_to_record[i - 1]
            msg_type_name = self.topics_to_record[i]

            if msg_type_name not in self.__msg_types_dict:
                self.get_logger().error(
                    f"msg type {msg_type_name} does not exist. Please adjust the topics array in \
                        globals.yaml"
                )

            self.__record_topics_sub.append(
                create_subscription_with_callback(topic_name, self.__msg_types_dict[msg_type_name])
            )

            if self.use_json:
                self.__data_to_write[topic_name] = None

            if self.use_bag:
                topic_info = rosbag2_py._storage.TopicMetadata(
                    name=topic_name, type=msg_type_name, serialization_format="cdr"
                )
                self.__writer.create_topic(topic_info)

    def __init_timer_callbacks(self):
        """Initializes timer callbacks of this node that are executed periodically."""
        self.get_logger().debug("Initializing timer callbacks...")
        self.create_timer(timer_period_sec=self.json_write_period, callback=self.__write_to_json)

    def __init_shutdown_callbacks(self):
        """Initializes shutdown callbacks of this node that are executed on shutdown."""
        self.get_logger().debug("Initializing shutdown callbacks...")
        self.context.on_shutdown(self.__shutdown_callback)

    # SUBSCRIPTION CALLBACKS
    def __general_sub_callback(self, msg: Type, topic_name: str):
        if self.use_json:
            msg_as_ord_dict = rosidl_runtime_py.message_to_ordereddict(msg)
            self.__data_to_write[topic_name] = msg_as_ord_dict

        if self.use_bag:
            self.__writer.write(
                topic_name, serialize_message(msg), self.get_clock().now().nanoseconds
            )

    # TIMER CALLBACKS
    def __write_to_json(self):
        # Only concern with this is if topic subscribed to is not launched, then we will never
        # write to json for all the other ones will never get written
        if all(self.__data_to_write.values()):  # all values are not None
            self.__data_to_write["time"] = self.get_clock().now().nanoseconds - self.__start_time
            if self.__json_index_counter > 0:
                self.__json_file.write(",\n")
            item_to_write = {self.__json_index_counter: self.__data_to_write}
            json.dump(item_to_write, self.__json_file, indent=4)
            self.__json_index_counter += 1

    # SHUTDOWN CALLBACKS
    def __shutdown_callback(self):
        """Shutdown callback to close bag and json."""
        self.get_logger().debug("Closing the storage files...")

        if self.use_json:
            self.__json_file.write("\n]")  # Close the JSON array
            self.__json_file.close()

        if self.use_bag:
            self.__writer.close()  # Needs to be called after json close to prevent early exit

    @property
    def file_name(self):
        return self.get_parameter("file_name").get_parameter_value().string_value

    @property
    def topics_to_record(self):
        return self.get_parameter("topics").get_parameter_value().string_array_value

    @property
    def use_bag(self):
        return self.get_parameter("bag").get_parameter_value().bool_value

    @property
    def use_json(self):
        return self.get_parameter("json").get_parameter_value().bool_value

    @property
    def json_write_period(self):
        return self.get_parameter("write_period_sec").get_parameter_value().double_value


if __name__ == "__main__":
    main()
