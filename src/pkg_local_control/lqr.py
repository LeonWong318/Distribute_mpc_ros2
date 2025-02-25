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
    def __init__(self, Ts, Q, R):
        """
        Initialize the LQR controller.
        
        :param Ts: Sampling time.
        :param Q: State weighting matrix.
        :param R: Input weighting matrix.
        """
        self.Ts = Ts
        self.Q = Q
        self.R = R

    def linearize(self, x_ref, u_ref):
        """
        Linearize the state-space model about the reference state and input.
        
        :param x_ref: Reference state [x, y, theta].
        :param u_ref: Reference input [v, omega].
        :return: Linearized matrices A and B.
        """
        theta_ref = x_ref[2]
        v_ref = u_ref[0]
        Ts = self.Ts
        
        # Jacobian of the dynamics with respect to the state
        A = np.array([
            [1, 0, -Ts * v_ref * np.sin(theta_ref)],
            [0, 1,  Ts * v_ref * np.cos(theta_ref)],
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
        
        :param x: Current state [x, y, theta].
        :param x_ref: Reference state [x_ref, y_ref, theta_ref].
        :param u_ref: Reference control input [v_ref, omega_ref].
        :return: Control input [v, omega], along with the LQR gain and closed-loop eigenvalues.
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

# Example usage:
# if __name__ == "__main__":
#     # Sampling time
#     Ts = 0.1

#     # Define Q and R matrices for the cost function
#     Q = np.diag([1.0, 1.0, 0.5])
#     R = np.diag([0.1, 0.1])
    
#     # Create an instance of the LQRController
#     lqr_controller = LQRController(Ts, Q, R)
    
#     # Define a reference trajectory point and corresponding control input.
#     # For example, at time k, the reference state and input might be:
#     x_ref = [5.0, 5.0, np.pi/4]  # desired state: x, y, theta
#     u_ref = [1.0, 0.0]           # desired control: v = 1 m/s, omega = 0 rad/s
    
#     # Current state of the vehicle
#     x = [4.5, 4.0, np.pi/3]
    
#     # Compute the LQR control input
#     u, K, P, eigVals = lqr_controller.compute_control(x, x_ref, u_ref)
    
#     print("Computed control input [v, omega]:", u)
#     print("LQR Gain K:\n", K)
#     print("Closed-loop eigenvalues:", eigVals)
