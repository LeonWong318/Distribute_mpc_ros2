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
        config = self._cfg
        self.Q = config.Q
        self.R = config.R
        self.d_safe = config.d_safe
        self.max_velocity = config.max_velocity
        self.output_dir = config.output_dir
    
    def build(self):
        """Build the CBF-LQR solver using OpenGen"""
        print(f"Building CBF-LQR solver")
        
        u = ca.SX.sym('u', 2)  # Control input [v, omega]
        p = ca.SX.sym('p', 11)  # Parameters [x, y, theta, x_ref, y_ref, theta_ref, v_ref, omega_ref, x_o, y_o, alpha]
        
        x, y, theta = p[0], p[1], p[2]
        x_ref, y_ref, theta_ref = p[3], p[4], p[5]
        v_ref, omega_ref = p[6], p[7]
        x_o, y_o, alpha = p[8], p[9], p[10]
        v, omega = u[0], u[1]
        
        # LQR Cost function
        state_error = ca.vertcat(x - x_ref, y - y_ref, theta - theta_ref)
        control_error = ca.vertcat(v - v_ref, omega - omega_ref)
        Q_mat = ca.diag(ca.vertcat(self.Q[0], self.Q[1], self.Q[2]))
        R_mat = ca.diag(ca.vertcat(self.R[0], self.R[1]))
        cost = ca.mtimes(control_error.T, ca.mtimes(R_mat, control_error)) + \
               ca.mtimes(state_error.T, ca.mtimes(Q_mat, state_error))
        
        # Corrected CBF constraint calculation
        h = self.d_safe**2 - (x - x_o)**2 - (y - y_o)**2
        dh_dt = -2 * (x - x_o) * v * ca.cos(theta) - 2 * (y - y_o) * v * ca.sin(theta)  # Fixed sign
        cbf_constraint = ca.vertcat(-(dh_dt + alpha * h))  # Flipped inequality for correct constraint
        
        # Define convex set C = (-∞, 0] for the constraint
        set_c = og.constraints.Rectangle([-ca.inf], [0.0])  # Correct set definition

        bounds = og.constraints.Rectangle([-self.max_velocity, -1.0], [self.max_velocity, 1.0])
        problem = og.builder.Problem(u, p, cost) \
            .with_constraints(bounds) \
            .with_aug_lagrangian_constraints(cbf_constraint, set_c)  # Now enforces dh_dt + alpha*h >= 0
        
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(self._cfg.tolerance) \
            .with_max_inner_iterations(self._cfg.max_inner_iterations) \
            .with_max_outer_iterations(self._cfg.max_outer_iterations) \
            .with_initial_penalty(self._cfg.initial_penalty) \
            .with_penalty_weight_update_factor(self._cfg.penalty_update_factor)
        
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name('cbf_solver') \
            .with_version('0.1.0')
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(self.output_dir) \
            .with_build_python_bindings() 
        

        builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)
           
        
        builder.build()
        
        return os.path.join(self.output_dir, meta.optimizer_name)