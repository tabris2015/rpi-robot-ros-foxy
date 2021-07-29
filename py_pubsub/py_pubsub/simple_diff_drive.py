from dataclasses import dataclass
from os import stat
import serial
from time import sleep
import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_ref_speed: float
    right_ref_speed: float
    left_speed:float
    right_speed: float
    left_effort: float
    right_effor: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float


class RobotNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('robot_node')
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.twist_subscription

        self.str_publisher = self.create_publisher(
            String,
            'robot_info',
            10
        )

        self.ser = serial.Serial('/dev/ttyACM0')
        self.get_logger().info(f'Using serial port {self.ser.name}')
    
    def send_command(self, linear: float, angular: float) -> SerialStatus:
        self.get_logger().info(f'Data to send: {linear}, {angular}')
        command = f'{linear:.3f},{angular:.3f}/'.encode('UTF-8')
        self.get_logger().info(f'Sending command: "{command}"')
        self.ser.write(command)
        while self.ser.in_waiting == 0:
            pass

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8')
        self.get_logger().info(f'data: "{res}", type: {type(res)}')
        if res == '0':
            return None
        raw_list = res.strip().split('/')[1].split(',')
        values_list = [float(value) for value in raw_list]
        return SerialStatus(*values_list)
    
    def twist_callback(self, twist: Twist):
        msg = String()
        self.get_logger().info(f'Received: {twist}')
        status = self.send_command(twist.linear.x, twist.angular.z)
        msg.data = f'data: {status}'
        self.get_logger().info(f'Status: {status}')
    

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()