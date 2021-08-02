from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class Lidar:
    def __init__(self):
        self.angles = np.array([])
        self.distances = np.array([])

    
    def update(self, node: Node, scan: LaserScan): 
        LOGGER = node.get_logger()

        LOGGER.info(f'Laser: [{scan.angle_min:.3f},{scan.angle_max:.3f},{scan.angle_increment:.3f}] ranges: {len(scan.ranges)}')

        self.angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        self.distances = np.array(scan.ranges)
        # filter inf numbers
        self.distances_valid = self.distances[self.distances < scan.range_max]
        self.distances[self.distances > scan.range_max] = scan.range_max
        
        LOGGER.info(f'range: [{scan.range_min:.3f},{scan.range_max:.3f}] -> [{np.min(self.distances):.3f},{np.max(self.distances):.3f}] -> [{np.min(self.distances_valid):.3f},{np.max(self.distances_valid):.3f}] => {len(self.distances_valid)}')
        LOGGER.info(f'angles: {len(self.angles)}, distances: {len(self.distances)}')
        