from os import times
import rclpy
import time
import math
import csv
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan

from rpi_robot_action_interfaces.action import GoToGoal
from rpi_robot_control.pid import PidController
from rpi_robot_control.lidar import Lidar




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

        self.get_logger().info("creating odometry subscriber...")
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.odom_subscription

        self.get_logger().info("creating laser subscriber...")
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        self.lidar_subscription
        self.lidar = Lidar(self)

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

        # transform
        self.tf_broadcaster = TransformBroadcaster(self)
        # important variables
        self.is_moving = False
        self.pos_tolerance = 0.05
        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0

        self.gtg_r = 0
        self.gtg_theta = 0
        self.ao_r = 0
        self.ao_theta = 0
        self.theta_desired = 0
        self.r_desired = 0

        self.sample_time = 0.1
        self.max_linear_v = 0.08
        self.alpha = 5
        self.blend_dist_threshold = 0.4 # m
        self.avoid_dist_threshold = 0.2
        self.avoid_angle_threshold = math.pi / 2.0
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
        # get direction vectors for both behaviors
        # self.get_logger().info('get "go-to-goal" vector')
        self.gtg_r, self.gtg_theta = self.get_go_to_goal_vector()
        # self.get_logger().info('get "avoid obstacle" vector')
        self.ao_r, self.ao_theta = self.get_obstacle_vector()

        self.publish_ref_vectors()
        

    def lidar_callback(self, scan: LaserScan):
        self.lidar.update(scan)

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
            self.get_logger().info(f'go_to_goal: [{self.gtg_r:.2f},{self.gtg_theta:.2f}], avoid obstacle: [{self.ao_r:.2f},{self.ao_theta:.2f}], desired: [{self.r_desired:.2f},{self.theta_desired:.2f}]')
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
        

        # TODO: blending
        self.r_desired = self.gtg_r
        # arctan2(r2sin(ϕ2−ϕ1),r1+r2cos(ϕ2−ϕ1))
        delta_theta = self.gtg_theta - self.ao_theta
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        
        if self.ao_r <= self.avoid_dist_threshold:
            # too close, avoid
            avoid_theta = self.ao_theta + math.pi
            avoid_theta = math.atan2(math.sin(avoid_theta), math.cos(avoid_theta))
            self.theta_desired = avoid_theta

        elif self.ao_r > self.avoid_dist_threshold and self.ao_r <= self.blend_dist_threshold and abs(delta_theta) < self.avoid_angle_threshold:
            # blend perpendicular
            avoid_theta = self.ao_theta + (math.copysign(1, delta_theta)) * (math.pi / 2.0) 
            avoid_theta = math.atan2(math.sin(avoid_theta), math.cos(avoid_theta))
            self.theta_desired = self.gtg_theta + math.atan2(self.ao_r * math.sin(self.ao_theta - self.gtg_theta), self.gtg_r + self.ao_r * math.cos(self.ao_theta - self.gtg_theta))
        
        else:
            self.theta_desired = self.gtg_theta

        #-------
        

        if self.get_distance_to_goal() <= self.pos_tolerance:
            self.linear_command = 0
            self.angular_command = 0
            self.get_logger().info(f"Already near goal, distance: {self.get_distance_to_goal()}")
            return True     # goal reached!

        # input to angle pid
        self.angle_pid.set_input(self.current_theta)
        # setpoint
        self.angle_pid.set_setpoint(self.theta_desired)

        # compute pid
        self.angle_pid.compute()

        self.linear_command = self.r_desired
        self.angular_command = self.angle_pid.get_output()      # from pid

        return False


    def get_go_to_goal_vector(self):
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        # self.get_logger().info(f'gtg error: ({error_x}, {error_y})')
        # compute K for linear velocity
        distance_to_goal = self.get_distance_to_goal()
        # self.get_logger().info(f'gtg distance: {distance_to_goal}')

        K = (self.max_linear_v * (1 - math.exp(-self.alpha * (distance_to_goal * distance_to_goal))) / (distance_to_goal * distance_to_goal)) if distance_to_goal > 0.0001 else 0

        # compute control signal
        u_x = K * error_x
        u_y = K * error_y

        # convert to polar coordinates 
        r = math.sqrt((u_x * u_x) + (u_y * u_y))
        theta = math.atan2(u_y, u_x)
        return r, theta

    def get_obstacle_vector(self):
        r_obs, theta_obs = self.lidar.get_closest_obstacle()
        return r_obs, theta_obs

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

    def quaternion_from_euler(self, roll, pitch, yaw) -> Quaternion:
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

    def send_twist_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_command)
        twist_msg.angular.z = float(self.angular_command)
        self.get_logger().info(f'Publishing twist: {twist_msg}')
        self.twist_publisher.publish(twist_msg)

    def publish_ref_vectors(self):
        timestamp = self.get_clock().now().to_msg()
        t_goal = TransformStamped()
        t_goal.header.stamp = timestamp
        t_goal.header.frame_id = '/base_link'
        t_goal.child_frame_id = '/gtg'

        t_goal.transform.rotation = self.quaternion_from_euler(0, 0, self.gtg_theta)

        t_obstacle = TransformStamped()
        t_obstacle.header.stamp = timestamp
        t_obstacle.header.frame_id = '/base_link'
        t_obstacle.child_frame_id = '/ao'

        t_obstacle.transform.rotation = self.quaternion_from_euler(0, 0, self.ao_theta)

        t_desired = TransformStamped()
        t_desired.header.stamp = timestamp
        t_desired.header.frame_id = '/base_link'
        t_desired.child_frame_id = '/desired'

        t_desired.transform.rotation = self.quaternion_from_euler(0, 0, self.theta_desired)
        # self.get_logger().info('Publishing transforms for vectors')
        self.tf_broadcaster.sendTransform(t_goal)
        self.tf_broadcaster.sendTransform(t_obstacle)
        self.tf_broadcaster.sendTransform(t_desired)


def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(go_to_goal_node, executor=executor)

    go_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()