import numpy as np

class PurePursuit:
    def __init__(self, lookahead_distance):
        """
        Initialize the Pure Pursuit controller.

        :param lookahead_distance: Distance to look ahead on the path.
        """
        self.lookahead_distance = lookahead_distance

    def find_lookahead_point(self, trajectory, current_position):
        """
        Find the lookahead point on the planned trajectory.

        :param trajectory: List of (x, y, theta) waypoints from MPC.
        :param current_position: Tuple (x, y) of the vehicle's current position.
        :return: The lookahead point (x, y, theta) or None if not found.
        """
        for point in trajectory:
            px, py, theta = point
            distance = np.linalg.norm(np.array([px, py]) - np.array(current_position))
            if distance >= self.lookahead_distance:
                return point  # Return the first valid lookahead point
        return None  # No valid lookahead point found

    def transform_to_vehicle_frame(self, current_position, current_heading, lookahead_point):
        """
        Transform the lookahead point to the vehicle's coordinate frame.

        :param current_position: (x, y) of the vehicle.
        :param current_heading: Current heading (radians).
        :param lookahead_point: (x, y, theta) from the trajectory.
        :return: (x', y') coordinates of the lookahead point in the vehicle's frame.
        """
        if lookahead_point is None:
            return None

        lx, ly, _ = lookahead_point

        # Compute transformation
        dx = lx - current_position[0]
        dy = ly - current_position[1]

        # Rotate the coordinates to vehicle frame
        x_vehicle = dx * np.cos(-current_heading) - dy * np.sin(-current_heading)
        y_vehicle = dx * np.sin(-current_heading) + dy * np.cos(-current_heading)

        return x_vehicle, y_vehicle

    def compute_control_commands(self, current_position, current_heading, v, lookahead_point):
        """
        Compute angular velocity (w) for the vehicle.

        :param current_position: (x, y) of the vehicle.
        :param current_heading: Current heading (radians).
        :param v: Current linear velocity.
        :param lookahead_point: (x, y, theta) of the lookahead point.
        :return: Angular velocity (w).
        """
        transformed_point = self.transform_to_vehicle_frame(current_position, current_heading, lookahead_point)
        if transformed_point is None:
            return 0.0  # No lookahead point, so keep moving straight

        x_vehicle, y_vehicle = transformed_point

        # Compute curvature (kappa)
        if abs(x_vehicle) < 1e-6:  # Avoid division by zero
            return 0.0

        curvature = 2 * y_vehicle / (self.lookahead_distance ** 2)

        # Compute angular velocity w = v * kappa
        w = v * curvature

        return w


