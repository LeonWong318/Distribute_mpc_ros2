import numpy as np
from rclpy.clock import Clock
from rclpy.time import Duration
import math

class PurePursuit:
    def __init__(self, lookahead_distance, Ts, v_max, alpha, lookahead_style, lookahead_time, mpc_ts):
        """
        Initialize the Pure Pursuit controller with state-space update parameters.

        :param lookahead_distance: Distance to look ahead on the path.
        :param Ts: Sampling time.
        :param v_max: Maximum velocity.
        :param alpha: Tuning parameter for velocity reduction at high curvature.
        :param lookahead_time: time.
        :param lookahead_style: style
        :param mpc_ts: mpc sampling time
        """
        self.lookahead_distance = lookahead_distance
        self.Ts = Ts
        self.v_max = v_max
        self.alpha = alpha
        self.lookahead_style = lookahead_style
        self.lookahead_time = lookahead_time
        self.mpc_ts = mpc_ts
    def is_forward(self, current_position, current_heading, trajectory_list):
        """
        Determine if the robot should drive forward or backward along the trajectory.

        Args:
            current_position: Tuple (x, y) of current position.
            current_heading: Current heading angle (theta) in radians.
            trajectory_list: List of trajectory points [(x, y, theta), ...].

        Returns:
            bool: True if should drive forward, False if backward.
        """
        if len(trajectory_list) < 2:
            return True  # Not enough points to infer direction — default to forward

        current_position = np.array(current_position)

        # Extract only the x, y components from the trajectory
        trajectory_xy = np.array([[x, y] for x, y, _ in trajectory_list])

        # Step 1: get the first point
        first_point = trajectory_xy[0]

        # Step 2: Get the check trajectory point
        check_point = trajectory_xy[5]
        traj_point = trajectory_xy[-1]

        # Step 3: Compute the direction vector from first point to check point
        path_direction = check_point - first_point
        if np.linalg.norm(path_direction) == 0:
            return True  # No direction — default to forward
        path_direction /= np.linalg.norm(path_direction)
        catch_direction = first_point-current_position
        catch_direction /= np.linalg.norm(catch_direction)
        traj_direction = traj_point - first_point
        traj_direction /= np.linalg.norm(traj_direction)
        # Step 4: Construct robot's heading vector
        heading_vector = np.array([np.cos(current_heading), np.sin(current_heading)])
        
        # Step 5: Use dot product to determine alignment
        check_dot = np.dot(heading_vector, path_direction)
        catch_dot = np.dot(heading_vector, catch_direction)
        traj_dot = np.dot(heading_vector, traj_direction)

        return check_dot >= 0, catch_dot >= 0, traj_dot >= 0 # True → forward, False → backward
    
    def find_lookahead_point(self, trajectory, current_position,current_heading, traj_time, current_time):
        """
        Find the lookahead point on the planned trajectory.
        
        :param trajectory: List of (x, y, theta) waypoints.
        :param current_position: Tuple (x, y) of the vehicle's current position.
        :param traj_time: the time when trajectory generated.
        :return: The lookahead point (x, y, theta) or None if not found.
        """
        current_state = np.array([current_position[0], current_position[1], current_heading])

        # Find the closest point and the look-ahead point
        min_dist = float('inf')
        closest_idx = 0
        look_ahead_idx = None
        path_forward, catch_forward, traj_forward = self.is_forward(current_position=current_position,current_heading=current_heading, trajectory_list=trajectory)
        if self.lookahead_style == 'dist':   
            if path_forward and (catch_forward or traj_forward):       
                # Forward condition  
                for i, point in enumerate(trajectory):
                    point_state = np.array([point[0], point[1]])
                    point_angle = math.atan2(point[1]-current_position[1], point[0]-current_position[0])
                    dist = np.linalg.norm(current_state[:2] - point_state)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                    # Compute the relative angle between the point and the current heading
                    relative_angle = point_angle - current_heading
                    # Normalize the angle to the range [-pi, pi]
                    relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
                    # Find the first point that is at least look_ahead_dist away
                    if dist >= self.look_ahead_dist and abs(relative_angle) <= math.pi / 2:
                        if look_ahead_idx is None:
                            look_ahead_idx = i
            return trajectory[look_ahead_idx]
        elif self.lookahead_style == 'time':
            
            target_time = current_time + self.lookahead_time
    
            index = int((target_time - traj_time) / self.mpc_ts)

            if 0 <= index < len(trajectory):
                return trajectory[index]
        return None

    def transform_to_vehicle_frame(self, current_position, current_heading, lookahead_point):
        """
        Transform the lookahead point to the vehicle's coordinate frame.
        
        :param current_position: Tuple (x, y) of the vehicle.
        :param current_heading: Current heading (radians).
        :param lookahead_point: (x, y, theta) of the lookahead point.
        :return: (x', y') coordinates of the lookahead point in the vehicle's frame.
        """
        if lookahead_point is None:
            return None

        lx, ly, _ = lookahead_point
        dx = lx - current_position[0]
        dy = ly - current_position[1]

        # Rotate the difference vector by -current_heading
        x_vehicle = dx * np.cos(-current_heading) - dy * np.sin(-current_heading)
        y_vehicle = dx * np.sin(-current_heading) + dy * np.cos(-current_heading)
        return x_vehicle, y_vehicle

    def compute_control_commands(self, current_position, current_heading, trajectory, traj_time, current_time):
        """
        Compute the control commands (v, omega) based on Pure Pursuit.
        
        :param current_position: (x, y) of the vehicle.
        :param current_heading: Current heading (radians).
        :param trajectory: List of (x, y, theta) waypoints.
        :param traj_time: the time when trajectory generated.
        :return: Control inputs: (v, omega).
        """
        # Find the lookahead point on the trajectory
        lookahead_point = self.find_lookahead_point(trajectory, current_position,current_heading, traj_time, current_time)
        if lookahead_point is None:
            # If no lookahead point is found, return zero control commands.
            return 0.0, 0.0, current_position

        # Transform the lookahead point into the vehicle's coordinate frame.
        transformed_point = self.transform_to_vehicle_frame(current_position, current_heading, lookahead_point)
        if transformed_point is None:
            return 0.0, 0.0, current_position

        # Only the lateral (y) coordinate is used to compute curvature.
        _, y_vehicle = transformed_point

        # Compute curvature: kappa = (2 * y_vehicle) / (L^2)
        curvature = (2 * y_vehicle) / (self.lookahead_distance ** 2)

        # Compute linear velocity: reduce speed when curvature is high.
        v = self.v_max * np.exp(-self.alpha * np.abs(curvature))

        # Compute angular velocity: omega = v * curvature.
        omega = v * curvature

        return v, omega, lookahead_point

    def update_state(self, state, control_input):
        """
        Update the state using the discrete state-space model.
        
        :param state: Current state as [x, y, theta].
        :param control_input: Control inputs as (v, omega).
        :return: Updated state as a numpy array.
        """
        x, y, theta = state
        v, omega = control_input

        x_next = x + self.Ts * v * np.cos(theta)
        y_next = y + self.Ts * v * np.sin(theta)
        theta_next = theta + self.Ts * omega

        return np.array([x_next, y_next, theta_next])


# # Example usage:
# if __name__ == "__main__":
#     # Define parameters
#     Ts = 0.1                   # Sampling time [s]
#     lookahead_distance = 2.0   # Lookahead distance [m]
#     v_max = 2.0                # Maximum velocity [m/s]
#     alpha = 1.0                # Tuning parameter for velocity reduction

#     # Instantiate the Pure Pursuit controller
#     pure_pursuit = PurePursuit(lookahead_distance, Ts, v_max, alpha)

#     # Define an example MPC trajectory as a list of (x, y, theta) waypoints.
#     trajectory = [
#         (0, 0, 0),
#         (2, 1, np.arctan2(1, 2)),
#         (4, 2, np.arctan2(1, 2)),
#         (6, 3, np.arctan2(1, 2)),
#         (8, 4, np.arctan2(1, 2))
#     ]

#     # Initial state: [x, y, theta]
#     state = np.array([0, 0, 0])

#     # Run the simulation for a number of steps.
#     for step in range(20):
#         current_position = state[:2]
#         current_heading = state[2]
#         # Compute control inputs (v, omega) based on the current state and trajectory.
#         v, omega = pure_pursuit.compute_control_commands(current_position, current_heading, trajectory)
#         control_input = (v, omega)
#         # Update the state using the computed control inputs.
#         state = pure_pursuit.update_state(state, control_input)
#         print(f"Step {step+1}: State: {state}, Control: v = {v:.2f} m/s, omega = {omega:.2f} rad/s")
