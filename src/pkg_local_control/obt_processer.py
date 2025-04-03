import numpy as np
from sensor_msgs.msg import LaserScan

class ObstacleProcessor:
    def __init__(self, safety_margin: float = 0.2, max_obstacle_distance: float = 0.8):
        """
        Process LaserScan data to find closest obstacles in map coordinates
        
        Args:
            safety_margin: Additional safety margin around obstacles (meters)
            max_obstacle_distance: Maximum distance to consider obstacles (meters)
        """
        self.safety_margin = safety_margin
        self.max_obstacle_distance = max_obstacle_distance
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Front laser position relative to the robot
        self.front_laser_offset = np.array([0.429646, 0.2358])
        self.front_laser_rotation = 0.785398  # 45 degrees in radians
        
        # Back laser position relative to the robot
        self.back_laser_offset = np.array([-0.354354, -0.2358])
        self.back_laser_rotation = -2.35619  # -135 degrees in radians
    
    def _filter_obstacles(self, obstacles, robot_pose):
        """
        Filter and sort obstacles by closest distance
        
        Args:
            obstacles: Array of obstacle positions in robot's coordinate frame
            robot_pose: Current robot pose [x, y, theta] in map coordinates
            
        Returns:
            Sorted obstacle positions and their distances
        """
        if obstacles.size == 0:
            return np.empty((0, 2)), np.empty(0)
        
        # Extract robot position
        robot_x, robot_y, _ = robot_pose
        
        # Calculate distances from robot position
        distances = np.linalg.norm(obstacles - [robot_x, robot_y], axis=1)
        
        # Apply safety margin adjustment
        adjusted_distances = distances - self.safety_margin
        
        # Filter obstacles that are too far
        valid_indices = adjusted_distances <= self.max_obstacle_distance
        if not np.any(valid_indices):
            return np.empty((0, 2)), np.empty(0)
        
        # Sort by adjusted distance (closest first)
        sorted_indices = np.argsort(adjusted_distances[valid_indices])
        filtered_obstacles = obstacles[valid_indices][sorted_indices]
        filtered_distances = adjusted_distances[valid_indices][sorted_indices]
        
        return filtered_obstacles, filtered_distances
    
    def get_closest_front_obstacles(self, sensor: LaserScan, robot_pose, num_obstacles: int = 1):
        """
        Get closest N front obstacles with safety margin considered
        """
        return self._get_closest_obstacles(sensor, robot_pose, num_obstacles, self.front_laser_offset, self.front_laser_rotation)
    
    def get_closest_back_obstacles(self, sensor: LaserScan, robot_pose, num_obstacles: int = 1):
        """
        Get closest N back obstacles with safety margin considered
        """
        return self._get_closest_obstacles(sensor, robot_pose, num_obstacles, self.back_laser_offset, self.back_laser_rotation)
    
    def _get_closest_obstacles(self, sensor: LaserScan, robot_pose, num_obstacles, laser_offset, laser_rotation):
        """
        Generic function to process laser scan data and return closest obstacles.
        """
        angles = np.linspace(sensor.angle_min, sensor.angle_max, len(sensor.ranges))
        ranges = np.array(sensor.ranges)
        range_min = 0.2
        range_max = self.max_obstacle_distance
        
        valid = (~np.isnan(ranges)) & (ranges >= range_min) & (ranges <= range_max)
        if not np.any(valid):
            return np.empty((0, 2)), np.empty(0)
        
        local_obstacles = np.vstack((
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid])
        )).T
        
        laser_cos, laser_sin = np.cos(laser_rotation), np.sin(laser_rotation)
        laser_rotation_matrix = np.array([[laser_cos, -laser_sin], [laser_sin, laser_cos]])
        local_obstacles = np.dot(local_obstacles, laser_rotation_matrix.T) + laser_offset
        
        robot_x, robot_y, robot_theta = robot_pose
        cos_theta, sin_theta = np.cos(robot_theta), np.sin(robot_theta)
        rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        map_obstacles = np.dot(local_obstacles, rotation.T) + np.array([robot_x, robot_y])
        
        closest_obstacles, distances = self._filter_obstacles(map_obstacles, robot_pose)
        
        n = min(num_obstacles, closest_obstacles.shape[0])
        return closest_obstacles[:n], distances
    
    def is_back_clear(self, back_obstacles, back_distances, min_safe_distance: float = 0.5):
        """
        Check if the back area is clear enough for a small backward movement
        using previously processed back obstacles
        
        Args:
            min_safe_distance: Minimum required distance to consider area safe (meters)
            
        Returns:
            bool: True if back area is clear for movement, False otherwise
        """
        # If no back obstacles detected, safe to move backward
        if back_obstacles.size == 0 or back_distances.size == 0:
            return True
        
        # Check if the closest back obstacle is beyond the minimum safe distance
        # Note: back_distances already has safety margin applied from _filter_obstacles
        if back_distances[0] >= min_safe_distance:
            return True
        
        return False