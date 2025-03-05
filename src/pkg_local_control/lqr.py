import numpy as np
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

class LQRController:
    def __init__(self, Ts, Q, R, max_velocity=1.0):
        """
        Initialize the LQR controller.
        
        Args:
            Ts: Sampling time.
            Q: State weighting matrix.
            R: Input weighting matrix.
            max_velocity: Maximum allowable velocity.
        """
        self.Ts = Ts
        self.Q = Q
        self.R = R
        self.max_velocity = max_velocity
    
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
    
    def compute_control_commands(self, current_position, current_heading, trajectory_list):
        """
        High-level method to compute control commands for trajectory tracking.
        Similar interface to Pure Pursuit for consistency.
        
        Args:
            current_position: Tuple (x, y) of current position.
            current_heading: Current heading angle (theta).
            trajectory_list: List of trajectory points [(x, y, theta), ...].
            
        Returns:
            v, omega: Linear and angular velocity commands.
        """
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
                    v_ref = direction_norm / self.Ts
                    theta_ref = (trajectory_list[next_idx][2] - x_ref[2]+ np.pi) % (2 * np.pi) - np.pi
                    omega_ref = theta_ref / self.Ts  # Simplified handling
                    
                    u_ref = np.array([v_ref, omega_ref])
                else:
                    u_ref = np.array([0.0, 0.0])
            else:
                u_ref = np.array([0.0, 0.0])
            
            # Compute LQR control
            u, _, _, _ = self.compute_control(current_state, x_ref, u_ref)
            
            # Return velocity commands
            return u[0], u[1]
            
        except Exception as e:
            print(f"Error in LQR compute_control_commands: {str(e)}")
            return 0.0, 0.0