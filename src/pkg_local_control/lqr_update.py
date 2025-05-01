import numpy as np
import math
from scipy.linalg import solve_discrete_are

def dlqr(A, B, Q, R):
    """
    Solve the discrete-time LQR controller for the system:
    x[k+1] = A x[k] + B u[k]
    with cost:
    J = sum { x[k]^T Q x[k] + u[k]^T R u[k] }.
    
    Returns:
    K: State feedback gain matrix.
    P: Solution to the discrete Riccati equation.
    eigVals: Eigenvalues of (A - B K).
    """
    # Solve the discrete-time algebraic Riccati equation
    P = solve_discrete_are(A, B, Q, R)
    
    # Compute the LQR gain
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    
    # Compute eigenvalues for closed-loop dynamics
    eigVals, _ = np.linalg.eig(A - B @ K)
    
    return K, P, eigVals

class LQR_Update_Controller:
    def __init__(self, Ts, Q, R, max_velocity, look_ahead_dist, lookahead_style, lookahead_time, mpc_ts):
        """
        Initialize the LQR controller.
        
        Args:
            Ts: Sampling time.
            Q: State weighting matrix.
            R: Input weighting matrix.
            max_velocity: Maximum allowable velocity.
            look_ahead_dist: the lookahead distance.
            lookahead_time: time.
            lookahead_style: style
            mpc_ts: mpc sampling time
        """
        self.Ts = Ts
        self.Q = Q
        self.R = R
        self.max_velocity = max_velocity
        self.look_ahead_dist = look_ahead_dist
        self.lookahead_style = lookahead_style
        self.lookahead_time = lookahead_time
        self.mpc_ts = mpc_ts
        # self.target_point = target_point
    
    def linearize(self, x_ref, u_ref):
        """
        Linearize the state-space model about the reference state and input.
        
        Args:
            x_ref: Reference state [x, y, theta].
            u_ref: Reference input [v, omega].
            
        Returns:
            A, B: Linearized system matrices.
        """
        theta_ref = x_ref[2]
        v_ref = u_ref[0]
        Ts = self.Ts
        
        # Jacobian of the dynamics with respect to the state
        A = np.array([
            [1, 0, -Ts * v_ref * np.sin(theta_ref)],
            [0, 1, Ts * v_ref * np.cos(theta_ref)],
            [0, 0, 1]
        ])
        
        # Jacobian of the dynamics with respect to the control input
        B = np.array([
            [Ts * np.cos(theta_ref), 0],
            [Ts * np.sin(theta_ref), 0],
            [0, Ts]
        ])
        
        return A, B
    
    def compute_control(self, x, x_ref, u_ref):
        """
        Compute the LQR control input for trajectory tracking.
        
        Args:
            x: Current state [x, y, theta].
            x_ref: Reference state [x_ref, y_ref, theta_ref].
            u_ref: Reference control input [v_ref, omega_ref].
            
        Returns:
            u: Control input [v, omega]
            K: LQR gain matrix
            P: Solution to the Riccati equation
            eigVals: Closed-loop eigenvalues
        """
        # Compute the error state (ensure angle error is wrapped between -pi and pi)
        error = np.array(x) - np.array(x_ref)
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        
        # Linearize the model around the reference
        A, B = self.linearize(x_ref, u_ref)
        
        # Compute the LQR gain using the discrete-time algebraic Riccati equation
        K, P, eigVals = dlqr(A, B, self.Q, self.R)
        
        # Compute the control correction
        delta_u = -K @ error
        
        # Final control input: reference input plus correction
        u = np.array(u_ref) + delta_u
        
        return u, K, P, eigVals
    
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
        check_point = trajectory_xy[10]
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

    def compute_control_commands(self, current_position, current_heading, trajectory_list, traj_time, current_time, target_piont):
        """
        Compute LQR control commands with a Pure Pursuit inspired look-ahead mechanism.

        Args:
            current_position: Tuple (x, y) of current position.
            current_heading: Current heading angle (theta).
            trajectory_list: List of trajectory points [(x, y, theta), ...].

        Returns:
            v, omega: Linear and angular velocity commands.
        """
        try:
            current_state = np.array([current_position[0], current_position[1], current_heading])
            if np.linalg.norm(target_piont - np.array([current_position[0], current_position[1]])) < 1:
                x_ref = np.array([
                    target_piont[0],
                    target_piont[1],
                    current_heading
                ])
                direction = target_piont - current_position
                direction_norm = np.linalg.norm(direction)

                
                v_ref = (direction[0] * np.cos(x_ref[2]) + direction[1] * np.sin(x_ref[2])) / self.Ts
                theta_ref = (trajectory_list[look_ahead_idx + 1][2] - x_ref[2] + np.pi) % (2 * np.pi) - np.pi
                omega_ref = theta_ref / self.Ts  
                u_ref = np.array([v_ref, omega_ref])
                
            else:
                # Find the closest point and the look-ahead point
                min_dist = float('inf')
                closest_idx = 0
                look_ahead_idx = None
                path_forward, catch_forward, traj_forward = self.is_forward(current_position=current_position,current_heading=current_heading, trajectory_list=trajectory_list)
                if self.lookahead_style == 'dist':   
                    if path_forward and (catch_forward or traj_forward):       
                        # Forward condition  
                        for i, point in enumerate(trajectory_list):
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
                    elif not path_forward and not (traj_forward and catch_forward):
                        # Backward condition
                        for i, point in enumerate(trajectory_list):
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
                            if dist >= self.look_ahead_dist and abs(relative_angle) >= math.pi / 2:
                                if look_ahead_idx is None:
                                    look_ahead_idx = i

                    else:
                        # find closest point as target
                        for i, point in enumerate(trajectory_list):
                            point_state = np.array([point[0], point[1]])
                            point_angle = math.atan2(point[1]-current_position[1], point[0]-current_position[0])
                            dist = np.linalg.norm(current_state[:2] - point_state)

                            if dist < min_dist:
                                min_dist = dist
                                closest_idx = i

                            # Find the first point that is at least look_ahead_dist away
                            if dist >= self.look_ahead_dist:
                                if look_ahead_idx is None:
                                    look_ahead_idx = i

                elif self.lookahead_style == 'time':

                    target_time = current_time + self.lookahead_time

                    look_ahead_idx = int((target_time - traj_time) / self.mpc_ts)
                    # Ensure the index is within bounds
                    if look_ahead_idx < 0:
                        look_ahead_idx = 0
                    elif look_ahead_idx >= len(trajectory_list):
                        look_ahead_idx = len(trajectory_list) - 1

             # Fallback to the last point if no look-ahead point is found
                if look_ahead_idx is None:
                    look_ahead_idx = len(trajectory_list) - 1

                # if the last point in trajectory is close enough then update to the last point
                last_point = np.array([trajectory_list[len(trajectory_list)-1][0], trajectory_list[len(trajectory_list)-1][1]])
                if np.linalg.norm(last_point - np.array([current_position[0], current_position[1]])) < 0.5:
                    print(f'the distance:{np.linalg.norm(last_point - current_position)}')
                    look_ahead_idx = len(trajectory_list)-1

                # Get reference state (using look-ahead index)
                x_ref = np.array([
                    trajectory_list[look_ahead_idx][0],
                    trajectory_list[look_ahead_idx][1],
                    trajectory_list[look_ahead_idx][2]
                ])

             # Compute reference control input
                if look_ahead_idx < len(trajectory_list) - 1:
                    next_point = np.array([
                        trajectory_list[look_ahead_idx + 1][0],
                        trajectory_list[look_ahead_idx + 1][1]
                    ])
                    current_point = np.array([
                        trajectory_list[look_ahead_idx][0],
                        trajectory_list[look_ahead_idx][1]
                    ])

                    direction = next_point - current_point
                    direction_norm = np.linalg.norm(direction)

                    if direction_norm > 1e-6:
                        v_ref = (direction[0] * np.cos(x_ref[2]) + direction[1] * np.sin(x_ref[2])) / self.Ts
                        theta_ref = (trajectory_list[look_ahead_idx + 1][2] - x_ref[2] + np.pi) % (2 * np.pi) - np.pi
                        omega_ref = theta_ref / self.Ts  
                        u_ref = np.array([v_ref, omega_ref])
                    else:
                        u_ref = np.array([0.0, 0.0])
                else:
                    u_ref = np.array([0.0, 0.0])

            # Compute LQR control
            u, _, _, _ = self.compute_control(current_state, x_ref, u_ref)

            return u[0], u[1], x_ref

        except Exception as e:
            print(f"Error in LQR compute_control_commands: {str(e)}")
            return u_ref[0], u_ref[1], x_ref
# def main():
#     # Sampling time and control horizon
#     Ts = 0.1
#     mpc_ts = 0.1

#     # Define LQR weights
#     Q = np.diag([10, 10, 5])
#     R = np.diag([1, 1])

#     # Define max velocity and look-ahead config
#     max_velocity = 1.0
#     look_ahead_dist = 1.0
#     lookahead_style = 'dist'  # or 'time'
#     lookahead_time = 1.0

#     # Create controller
#     controller = LQR_Update_Controller(Ts, Q, R, max_velocity, look_ahead_dist, lookahead_style, lookahead_time, mpc_ts)

#     # Create a simple trajectory (straight line forward)
#     trajectory = [(i * 0.2, 0.0, 0.0) for i in range(20)]  # (x, y, theta)

#     # Set initial robot state
#     current_position = (0.0, 0.1)  # slightly off the path
#     current_heading = 0.0
#     traj_time = 0.0
#     current_time = 0.0

#     # Run the control computation
#     v, omega, x_ref = controller.compute_control_commands(
#         current_position, current_heading, trajectory, traj_time, current_time
#     )

#     # Output results
#     print(f"Computed control commands:\n  v = {v:.3f} m/s\n  omega = {omega:.3f} rad/s")
#     print(f"Tracking reference: {x_ref}")

# if __name__ == "__main__":
#     main()
