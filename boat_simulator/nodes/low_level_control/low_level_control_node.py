#!/usr/bin/env python3

import sys
from typing import Optional

import rclpy
import rclpy.utilities
from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation
from custom_interfaces.msg import GPS
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import (
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    Node,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Rate

import boat_simulator.common.constants as Constants
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.low_level_control.decorators import (
    MutuallyExclusiveActionRoutine,
)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


class LowLevelControlNode(Node):
    def __init__(self):
        super().__init__("low_level_control_node")

        self.get_logger().debug("Initializing node...")

        self.__rudder_angle = 0
        self.__sail_trim_tab_angle = 0
        self.__desired_heading = None
        self._is_rudder_action_active = False
        self.__is_sail_action_active = False

        self.__declare_ros_parameters()
        self.__init_callback_groups()
        self.__init_feedback_execution_rates()
        self.__init_subscriptions()
        self.__init_action_servers()

        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __declare_ros_parameters(self):
        # TODO Update global YAML file with more configuration parameters and declare them here
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        )

    def __init_callback_groups(self):
        self.__pub_sub_callback_group = MutuallyExclusiveCallbackGroup()
        self.__rudder_action_callback_group = ReentrantCallbackGroup()
        self.__sail_action_callback_group = MutuallyExclusiveCallbackGroup()

    def __init_feedback_execution_rates(self):
        self.__rudder_action_feedback_rate = self.create_rate(
            frequency=Constants.RUDDER_ACTUATION_EXECUTION_PERIOD_SEC, clock=self.get_clock()
        )
        self.__sail_action_feedback_rate = self.create_rate(
            Constants.SAIL_TRIM_TAB_ACTUATION_PERIOD_SEC, clock=self.get_clock()
        )

    def __init_subscriptions(self):
        self.get_logger().debug("Initializing subscriptions...")

        self.__gps_sub = self.create_subscription(
            msg_type=GPS,
            topic=Constants.LOW_LEVEL_CTRL_SUBSCRIPTIONS.GPS,
            callback=self.__gps_sub_callback,
            qos_profile=Constants.QOS_DEPTH,
            callback_group=self.pub_sub_callback_group,
        )

        self.get_logger().debug("Done initializing subscriptions...")

    def __init_action_servers(self):
        self.__rudder_actuation_action_server = ActionServer(
            node=self,
            action_type=SimRudderActuation,
            action_name=Constants.ACTION_NAMES.RUDDER_ACTUATION,
            execute_callback=self.__rudder_actuation_routine,
            callback_group=self.rudder_action_callback_group,
        )

    def __gps_sub_callback(self, msg: GPS):
        self.__gps_sub = msg

    @MutuallyExclusiveActionRoutine(action_type=SimRudderActuation)
    def __rudder_actuation_routine(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Beginning rudder actuation")

        feedback_msg = SimRudderActuation.Feedback()
        for i in range(Constants.RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS):
            feedback_msg.rudder_angle = float(i)
            self.get_logger().fatal(f"Server feedback: {i}")
            self.get_logger().fatal(f"Is Action Active? {self.is_rudder_action_active}")
            goal_handle.publish_feedback(feedback=feedback_msg)
            self.rudder_action_feedback_rate.sleep()

        goal_handle.succeed()

        result = SimRudderActuation.Result()
        result.remaining_angular_distance = 0.0
        return result

    @property
    def is_multithreading_enabled(self) -> bool:
        return self.__is_multithreading_enabled

    @property
    def pub_sub_callback_group(self) -> CallbackGroup:
        return self.__pub_sub_callback_group

    @property
    def rudder_action_callback_group(self) -> CallbackGroup:
        return self.__rudder_action_callback_group

    @property
    def sail_action_callback_group(self) -> CallbackGroup:
        return self.__sail_action_callback_group

    @property
    def rudder_actuation_action_server(self) -> ActionServer:
        return self.__rudder_actuation_action_server

    @property
    def sail_actuation_action_server(self) -> ActionServer:
        return self.__sail_actuation_action_server

    @property
    def rudder_action_feedback_rate(self) -> Rate:
        return self.__rudder_action_feedback_rate

    @property
    def sail_action_feedback_rate(self) -> Rate:
        return self.__sail_action_feedback_rate

    @property
    def is_rudder_action_active(self) -> bool:
        return self._is_rudder_action_active

    @property
    def is_sail_action_active(self) -> bool:
        return self.__is_sail_action_active

    @property
    def pub_period(self) -> float:
        return self.get_parameter("pub_period_sec").get_parameter_value().double_value

    @property
    def gps_sub(self) -> Subscription:
        return self.__gps_sub

    @property
    def rudder_angle(self) -> Scalar:
        return self.__rudder_angle

    @property
    def sail_trim_tab_angle(self) -> Scalar:
        return self.__sail_trim_tab_angle


if __name__ == "__main__":
    main()
