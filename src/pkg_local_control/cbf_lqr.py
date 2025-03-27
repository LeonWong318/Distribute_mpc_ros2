import numpy as np
import os
import sys
from sensor_msgs.msg import LaserScan
from pkg_configs.configs import CBFconfig
from typing import List

class Solver(): 
    import opengen as og 
    def run(self, p: List, initial_guess=None, initial_lagrange_multipliers=None, initial_penalty=None) -> og.opengen.tcp.solver_status.SolverStatus: pass

class ObstacleProcessor:
    def __init__(self, sensor: LaserScan, robot_pose, safety_margin: float = 0.2, max_obstacle_distance: float = 3.0):
        """ 
        Process LaserScan data to find closest obstacles in map coordinates
        
        :param sensor: LaserScan message from ROS
        :param robot_pose: Current robot pose [x, y, theta] in map coordinates
        :param safety_margin: Additional safety margin around obstacles (meters)
        :param max_obstacle_distance: Maximum distance to consider obstacles (meters)
        """
        # Robot's current position and heading
        self.robot_x, self.robot_y, self.robot_theta = robot_pose
        
        # Convert polar coordinates to Cartesian
        angles = np.linspace(sensor.angle_min, sensor.angle_max, len(sensor.ranges))
        ranges = np.array(sensor.ranges)
        
        # Filter valid measurements
        valid = (~np.isnan(ranges)) & (ranges >= sensor.range_min) & (ranges <= min(sensor.range_max, max_obstacle_distance))
        
        # Convert laser scan to robot's local coordinate frame
        local_obstacles = np.vstack((
            ranges[valid] * np.cos(angles[valid]), 
            ranges[valid] * np.sin(angles[valid])
        )).T  # Shape: (N, 2)
        
        # Transform local obstacles to map coordinates
        # Rotation matrix for robot's heading
        rotation_matrix = np.array([
            [np.cos(self.robot_theta), -np.sin(self.robot_theta)],
            [np.sin(self.robot_theta), np.cos(self.robot_theta)]
        ])
        
        # Transform obstacles to map coordinates
        self.obstacles = local_obstacles @ rotation_matrix.T + [self.robot_x, self.robot_y]
        
        # Apply safety margin
        self.safety_margin = safety_margin
        self._filter_obstacles()
    
    def _filter_obstacles(self):
        """Filter and sort obstacles by distance"""
        if self.obstacles.size == 0:
            self.closest_obstacles = np.empty((0, 2))
            return
        
        # Calculate distances from map origin
        distances = np.linalg.norm(self.obstacles - [self.robot_x, self.robot_y], axis=1)
        
        # Apply safety margin adjustment
        adjusted_distances = distances - self.safety_margin
        
        # Sort by adjusted distance
        sorted_indices = np.argsort(adjusted_distances)
        self.closest_obstacles = self.obstacles[sorted_indices]
        self.distances = adjusted_distances[sorted_indices]
    
    def get_closest_obstacles(self, num_obstacles: int = 3):
        """ 
        Get closest N obstacles with safety margin considered
        
        :param num_obstacles: Number of closest obstacles to return
        :return: Array of obstacle positions in map coordinates (shape: (N, 2))
        """
        if self.closest_obstacles.shape[0] == 0:
            return np.empty((0, 2))
        
        n = min(num_obstacles, self.closest_obstacles.shape[0])
        return self.closest_obstacles[:n]
    
    def get_most_dangerous_obstacle(self):
        """ 
        Get the single most dangerous/closest obstacle
        
        :return: (x, y) position in map coordinates or None if no obstacles 
        """
        if self.closest_obstacles.shape[0] == 0:
            return None
        
        return self.closest_obstacles[0]
    
class CBF_LQR_Controller:
    def __init__(self, config : CBFconfig, max_velocity=1.0, Ts=0.1):
        self.Q = config.Q
        self.R = config.R
        self.d_safe = config.d_safe
        self.max_velocity = max_velocity
        self.Ts = Ts
        self.config = config
        # Initialize solver from Python bindings
        self.solver_path = os.path.join(self.config.output_dir,self.config.solver_name)
        sys.path.append(self.solver_path)
        self.build_solver =  __import__(self.config.solver_name)
        self.solver:Solver = self.build_solver.solver()

    def compute_control(self, x, x_ref, u_ref, obstacles):
        if obstacles.shape[1] == 0:
            return u_ref[0], u_ref[1]

        try:
            # Find nearest obstacle
            x_o, y_o = obstacles[0]
            alpha = self.config.alpha  # CBF parameter

            # Build parameter vector [x, y, theta, x_ref, y_ref, theta_ref, 
            #                        v_ref, omega_ref, x_o, y_o, alpha]
            params = np.concatenate([
                x,                # current state (3)
                x_ref,            # reference state (3)
                u_ref,            # reference control (2)
                [x_o, y_o, alpha] # obstacle + CBF param (3)
            ])

            # Solve using Python bindings
            solution = self.solver.run(p=params)
            
            # Extract solution - format depends on your problem setup
            if solution['exit_status'] == 'Converged':
                return solution['solution'][0], solution['solution'][1]
            
            
            return float(u_ref[0]), float(u_ref[1])

        except Exception as e:
            print(f"Control computation failed: {str(e)}")
            return 0.0, 0.0
    
    def compute_control_commands(self, current_position, current_heading, 
                                trajectory_list, obstacles):
        try:
            current_state = np.array([
                current_position[0], 
                current_position[1], 
                current_heading
            ])

            # Simplified reference tracking (same as before)
            closest_idx = np.argmin([
                np.linalg.norm(point[:2] - current_state[:2])
                for point in trajectory_list
            ])
            x_ref = np.array(trajectory_list[closest_idx])

            # Simplified reference control calculation
            if closest_idx < len(trajectory_list) - 1:
                next_point = trajectory_list[closest_idx + 1]
                direction = next_point[:2] - x_ref[:2]
                v_ref = np.linalg.norm(direction) / self.Ts
                omega_ref = (next_point[2] - x_ref[2]) / self.Ts
            else:
                v_ref, omega_ref = 0.0, 0.0

            # Compute safe control
            return self.compute_control(
                current_state,
                x_ref,
                np.array([v_ref, omega_ref]),
                obstacles
            )

        except Exception as e:
            print(f"Control pipeline error: {str(e)}")
            return 0.0, 0.0
