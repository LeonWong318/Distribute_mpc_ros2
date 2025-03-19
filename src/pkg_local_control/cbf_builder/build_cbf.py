import os
import casadi as ca
import opengen as og
from pkg_configs.configs import CBFconfig

class CBF_builder:
    """This is the CBF builder"""
    
    def __init__(self, cbf_config: CBFconfig):
        self._cfg = cbf_config
        self._load_variables()
    
    @classmethod
    def from_yaml(cls, mpc_cfg_fpath: str, robot_cfg_fpath: str):
        cbf_config = CBFconfig.from_yaml(mpc_cfg_fpath)
        return cls(cbf_config)
    
    def _load_variables(self):
        """Load configuration variables from the config object"""
        # Extract parameters from config
        config = self._cfg
        
        # Control parameters
        self.Q = config.Q
        self.R = config.R
        self.d_safe = config.d_safe
        self.max_velocity = config.max_velocity
        
        # Output directory
        self.output_dir = config.output_dir
    
    def build(self):
        """Build the CBF-LQR solver using OpenGen"""
        print(f"Building CBF-LQR solver")
        
        # Define optimization variables
        u = ca.SX.sym('u', 2)  # Control input [v, omega]
        p = ca.SX.sym('p', 10)  # Parameters [x, y, theta, x_ref, y_ref, v_ref, omega_ref, x_o, y_o, alpha]
        
        # Extract parameters
        x, y, theta = p[0], p[1], p[2]
        x_ref, y_ref = p[3], p[4]
        v_ref, omega_ref = p[5], p[6]
        x_o, y_o, alpha = p[7], p[8], p[9]
        v, omega = u[0], u[1]
        
        # LQR Cost function
        state_error = ca.vertcat(x - x_ref, y - y_ref)
        control_error = ca.vertcat(v - v_ref, omega - omega_ref)
        Q_mat = ca.diag(ca.vertcat(self.Q[0], self.Q[1], self.Q[2]))
        R_mat = ca.diag(ca.vertcat(self.R[0], self.R[1]))
        cost = ca.mtimes(ca.mtimes(control_error.T, R_mat), control_error) + \
               ca.mtimes(ca.mtimes(state_error.T, Q_mat), state_error)
        
        # CBF constraint
        h = self.d_safe**2 - (x - x_o)**2 - (y - y_o)**2
        dh_dt = 2 * (x - x_o) * v * ca.cos(theta) + 2 * (y - y_o) * v * ca.sin(theta)
        cbf_constraint = dh_dt + alpha * h
        
        # Set up optimization problem
        bounds = og.constraints.Rectangle([-self.max_velocity, -1.0], [self.max_velocity, 1.0])
        problem = og.builder.Problem(u, p, cost) \
            .with_constraints(og.constraints.Inequality(cbf_constraint)) \
            .with_aug_lagrangian_constraints(bounds)
        
        # Configure solver
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(1e-4) \
            .with_max_inner_iterations(500) \
            .with_max_outer_iterations(10) \
            .with_initial_penalty(1.0) \
            .with_penalty_weight_update_factor(5.0)
        
        # Meta information
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name('cbf_lqr_controller') \
            .with_version('0.1.0')
        
        # Build problem
        builder = og.builder.OpEnOptimizerBuilder(problem, meta, solver_config) \
            .with_build_directory(self.output_dir) \
            .with_build_mode('release') \
            .with_tcp_interface_config()
        
        # Build solver
        builder.build()
        
        return os.path.join(self.output_dir, meta.optimizer_name)