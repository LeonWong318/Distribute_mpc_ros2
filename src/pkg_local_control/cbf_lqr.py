import numpy as np
from sensor_msgs.msg import LaserScan
import opengen as og

def process_laserscan(scan_msg):
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
    ranges = np.array(scan_msg.ranges)

    # Convert to (x, y) coordinates (ignoring NaN or inf values)
    valid_indices = ~np.isnan(ranges) & (ranges < scan_msg.range_max)
    x_obstacles = ranges[valid_indices] * np.cos(angles[valid_indices])
    y_obstacles = ranges[valid_indices] * np.sin(angles[valid_indices])

    return np.vstack((x_obstacles, y_obstacles))  # Shape (2, N)

class CBF_LQR_Controller:
    def __init__(self, Q, R, d_safe, max_velocity=1.0, Ts=0.1):
        self.Q = Q
        self.R = R
        self.d_safe = d_safe
        self.max_velocity = max_velocity
        self.Ts = Ts  # Added timestep parameter

        # Define the optimizer
        self.optimizer = self.build_optimizer()

    
    def compute_control(self, x, x_ref, u_ref, obstacles):
        """
        Solve the QP for the optimal control input.
        """
        if obstacles.shape[1] == 0:
            return u_ref  # No obstacles, use nominal control

        x_o, y_o = obstacles[:, np.argmin(np.linalg.norm(obstacles - x[:2, None], axis=0))]

        params = np.hstack([x, x_ref, u_ref, x_o, y_o, 0.5])  # Alpha = 0.5
        solution = self.optimizer.solve(params)
        return solution
    
    def compute_control_commands(self, current_position, current_heading, trajectory_list, obstacles):
        try:
            # Convert inputs to numpy arrays
            current_state = np.array([current_position[0], current_position[1], current_heading])
            
            # Find closest point on trajectory
            min_dist = float('inf')
            closest_idx = 0
            
            for i, point in enumerate(trajectory_list):
                point_state = np.array([point[0], point[1], point[2]])
                dist = np.linalg.norm(current_state[:2] - point_state[:2])
                
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            # Find next point for reference
            next_idx = min(closest_idx + 1, len(trajectory_list) - 1)
            
            # Get reference state
            x_ref = np.array([
                trajectory_list[closest_idx][0],
                trajectory_list[closest_idx][1],
                trajectory_list[closest_idx][2]
            ])
            
            # Compute reference control input
            if closest_idx < len(trajectory_list) - 1:
                next_point = np.array([
                    trajectory_list[next_idx][0],
                    trajectory_list[next_idx][1]
                ])
                current_point = np.array([
                    trajectory_list[closest_idx][0],
                    trajectory_list[closest_idx][1]
                ])
                
                direction = next_point - current_point
                direction_norm = np.linalg.norm(direction)
                
                if direction_norm > 1e-6:
                    v_ref = (direction[0] * np.cos(x_ref[2]) + direction[1] * np.sin(x_ref[2])) / self.Ts
                    theta_ref = (trajectory_list[next_idx][2] - x_ref[2]+ np.pi) % (2 * np.pi) - np.pi
                    omega_ref = theta_ref / self.Ts  # Simplified handling
                    
                    u_ref = np.array([v_ref, omega_ref])
                else:
                    u_ref = np.array([0.0, 0.0])
            else:
                u_ref = np.array([0.0, 0.0])
            
            # Compute control with obstacles
            u = self.compute_control(current_state, x_ref, u_ref, obstacles)
            
            # Return velocity commands
            return u[0], u[1]
            
        except Exception as e:
            print(f"Error in CBF-LQR compute_control_commands: {str(e)}")
            return 0.0, 0.0