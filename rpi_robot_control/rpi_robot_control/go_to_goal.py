import rclpy
import time
import math
import csv
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rpi_robot_action_interfaces.action import GoToGoal
from rpi_robot_control.pid import PidController


class GoToGoalNode(Node):
    """Node for high level control of a differential drive robot"""
    def __init__(self):
        
        super().__init__('go_to_goal_node')
        self.get_logger().info("creating publisher...")
        self.twist_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.get_logger().info("creating subscriber...")
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.odom_subscription

        self.get_logger().info("creating action server...")
        self.go_to_goal_action_service = ActionServer(
            self,
            GoToGoal,
            'go_to_goal_service',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # important variables
        self.is_moving = False
        self.pos_tolerance = 0.05
        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0

        self.sample_time = 0.1
        self.max_linear_v = 0.08
        self.alpha = 5
        self.angle_pid = PidController(0.3, 0.01, 0.002, self.sample_time, True)
        self.angle_pid.set_output_limits(-3, 3)

        self.get_logger().info("initialization finished")

        self.linear_command = 0
        self.angular_command = 0

        ## logging utils
        

    def odom_callback(self, odom: Odometry):
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        self.current_theta = self.get_euler_from_quaternion(odom.pose.pose.orientation)[2]
        # self.get_logger().info(f'Current Position: ({self.current_x}, {self.current_y}, {self.current_theta})')


    def add_two_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request a:{request.a}, b:{request.b}')
        return response

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        if self.is_moving:
            return GoalResponse.REJECT

        self.goal_x = goal_request.x
        self.goal_y = goal_request.y
        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Distance to goal: {self.get_distance_to_goal()}')
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        # TODO: implement logic to cancel request
        return CancelResponse.ACCEPT

    # async method
    async def execute_callback(self, goal_handle):
        """Execute a goal"""
        self.get_logger().info('Executing action...')

        feedback_msg = GoToGoal.Feedback()
        log_file = open(f'/home/pepe/log/{int(time.time() * 1000)}.log', 'w')

        while not self.update_control_loop():
            self.is_moving = True
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_x
            feedback_msg.distance = self.get_distance_to_goal()
            self.get_logger().info(f'current pos: {feedback_msg.current_x},{feedback_msg.current_y}, distance to goal: {self.get_distance_to_goal()}')
            self.send_twist_command()
            goal_handle.publish_feedback(feedback_msg)
            log_str = f'{int(time.time() * 1000)},{self.current_x},{self.current_y},{self.get_distance_to_goal()},{self.linear_command},{self.angular_command}\n'
            log_file.write(log_str)
            time.sleep(self.sample_time)

        self.is_moving = False
        # should be 0
        self.send_twist_command()
        log_str = f'{int(time.time() * 1000)},{self.current_x},{self.current_y},{self.get_distance_to_goal()},{self.linear_command},{self.angular_command}\n'
        log_file.write(log_str)
        log_file.close()
        goal_handle.succeed()
        result = GoToGoal.Result()
        result.goal_reached = True
        self.get_logger().info(f'Returning result: {result.goal_reached}')
        return result


    def update_control_loop(self):
        # compute error
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y

        # compute K for linear velocity
        distance_to_goal = self.get_distance_to_goal()

        if distance_to_goal <= self.pos_tolerance:
            self.linear_command = 0
            self.angular_command = 0
            self.get_logger().info(f"Already near goal, distance: {distance_to_goal}")
            return True     # goal reached!

        
        K = self.max_linear_v * (1 - math.exp(-self.alpha * (distance_to_goal * distance_to_goal))) / (distance_to_goal * distance_to_goal)

        # compute control signal
        u_x = K * error_x
        u_y = K * error_y

        # input to angle pid
        self.angle_pid.set_input(self.current_theta)
        # setpoint
        theta_goal = math.atan2(u_y, u_x)
        self.angle_pid.set_setpoint(theta_goal)

        # compute pid
        self.angle_pid.compute()

        self.linear_command = math.sqrt((u_x * u_x) + (u_x * u_x))
        self.angular_command = self.angle_pid.get_output()

        return False

    

    def get_distance_to_goal(self):
        return math.sqrt(math.pow(self.goal_x - self.current_x, 2) + math.pow(self.goal_y - self.current_y, 2))

    def get_euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def send_twist_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_command)
        twist_msg.angular.z = float(self.angular_command)
        self.get_logger().info(f'Publishing twist: {twist_msg}')
        self.twist_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(go_to_goal_node, executor=executor)

    go_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()