#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicsEngineNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'physics_engine_out', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World! %d' % self.counter_
        self.publisher_.publish(msg)
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsEngineNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
