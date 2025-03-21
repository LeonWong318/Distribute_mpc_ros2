import numpy as np

class PurePursuit:
    def __init__(self, lookahead_distance, Ts, v_max, alpha, lookahead_style, lookahead_time):
        """
        Initialize the Pure Pursuit controller with state-space update parameters.

        :param lookahead_distance: Distance to look ahead on the path.
        :param Ts: Sampling time.
        :param v_max: Maximum velocity.
        :param alpha: Tuning parameter for velocity reduction at high curvature.
        :param lookahead_time: time.
        :param lookahead_style: style
        """
        self.lookahead_distance = lookahead_distance
        self.Ts = Ts
        self.v_max = v_max
        self.alpha = alpha
        self.lookahead_style = lookahead_style
        self.lookahead_time = lookahead_time

    def find_lookahead_point(self, trajectory, current_position):
        """
        Find the lookahead point on the planned trajectory.
        
        :param trajectory: List of (x, y, theta) waypoints.
        :param current_position: Tuple (x, y) of the vehicle's current position.
        :return: The lookahead point (x, y, theta) or None if not found.
        """
        if self.lookahead_style == 'dist':
            for point in trajectory:
                px, py, theta = point
                distance = np.linalg.norm(np.array([px, py]) - np.array(current_position))
                if distance >= self.lookahead_distance:
                    return point
        elif self.lookahead_style == 'time':
            
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

    def compute_control_commands(self, current_position, current_heading, trajectory):
        """
        Compute the control commands (v, omega) based on Pure Pursuit.
        
        :param current_position: (x, y) of the vehicle.
        :param current_heading: Current heading (radians).
        :param trajectory: List of (x, y, theta) waypoints.
        :return: Control inputs: (v, omega).
        """
        # Find the lookahead point on the trajectory
        lookahead_point = self.find_lookahead_point(trajectory, current_position)
        if lookahead_point is None:
            # If no lookahead point is found, return zero control commands.
            return 0.0, 0.0

        # Transform the lookahead point into the vehicle's coordinate frame.
        transformed_point = self.transform_to_vehicle_frame(current_position, current_heading, lookahead_point)
        if transformed_point is None:
            return 0.0, 0.0

        # Only the lateral (y) coordinate is used to compute curvature.
        _, y_vehicle = transformed_point

        # Compute curvature: kappa = (2 * y_vehicle) / (L^2)
        curvature = (2 * y_vehicle) / (self.lookahead_distance ** 2)

        # Compute linear velocity: reduce speed when curvature is high.
        v = self.v_max * np.exp(-self.alpha * np.abs(curvature))

        # Compute angular velocity: omega = v * curvature.
        omega = v * curvature

        return v, omega

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
