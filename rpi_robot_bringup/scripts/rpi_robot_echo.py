#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class EchoNode(Node):
    """Simple node for echoing an incoming message"""
    def __init__(self):
        super().__init__('robot_node')
        self.subscription = self.create_subscription(
            String,
            'rpi_msg',
            self.string_callback,
            10
        )
        self.subscription

        self.str_publisher = self.create_publisher(
            String,
            'rpi_echo',
            10
        )
    
    def string_callback(self, string: String):
        msg = String()
        self.get_logger().info(f'Received: {string.data}')
        msg.data = f'echo: {string.data}'
    

def main(args=None):
    rclpy.init(args=args)
    echo_node = EchoNode()
    rclpy.spin(echo_node)

    echo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()