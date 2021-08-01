import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class GoToGoalNode(Node):
    """Node for high level control of a differential drive robot"""
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.twist_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.odom_subscription

    def odom_callback(self, odom: Odometry):
        self.get_logger().info(f'Current Position: ({odom.pose.pose.position.x}, {odom.pose.pose.position.y})')

    

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)

    go_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()