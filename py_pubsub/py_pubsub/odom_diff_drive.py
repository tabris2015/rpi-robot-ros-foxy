from dataclasses import dataclass
from os import stat
import math
import serial
from time import sleep
import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q


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

        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
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
        odom_msg = Odometry()
        self.get_logger().info(f'Received: {twist}')
        status = self.send_command(twist.linear.x, twist.angular.z)
        if status is None:
            return
        odom_msg.pose.pose.position.x = status.x_pos
        odom_msg.pose.pose.position.y = status.y_pos
        odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, status.theta)
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(f'Status: {status}')

        

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()