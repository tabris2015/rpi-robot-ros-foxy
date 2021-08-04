from numpy.core.numeric import Inf
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Lidar:
    def __init__(self, node: Node):
        self.angles = np.array([])
        self.distances = np.array([])

        self.theta_obstacle = 0
        self.r_obstacle = 0   
        self.node = node
        self.LOGGER = self.node.get_logger()     

    
    def update(self, scan: LaserScan): 
        # self.LOGGER.info(f'Laser: [{scan.angle_min:.3f},{scan.angle_max:.3f},{scan.angle_increment:.3f}] ranges: {len(scan.ranges)}')

        self.angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)# * (180.0 / math.pi)
        self.distances = np.array(scan.ranges)
        # filter inf numbers
        if len(self.distances) != len(self.angles):
            return

        # self.LOGGER.info(f'angles: {len(self.angles)}, distances: {len(self.distances)}')
        # self.LOGGER.info(f'min distance: ({self.angles[np.argmin(self.distances)]}deg,{np.min(self.distances)}m) - id: {np.argmin(self.distances)}')
        
        # angle and distance relative to robot
        self.theta_obstacle = self.angles[np.argmin(self.distances)]
        self.r_obstacle = np.min(self.distances)
        
    def get_closest_obstacle(self):
        # self.LOGGER.info(f'obstacle: {self.theta_obstacle * (180.0 / math.pi)} [deg] {self.r_obstacle} [m]')
        return self.r_obstacle, self.theta_obstacle
        
