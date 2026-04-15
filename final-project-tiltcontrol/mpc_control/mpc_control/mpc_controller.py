import cvxpy as cp
import numpy as np
import rclpy
from scipy.signal import cont2discrete

from control_base.base_controller import BaseController


class MPCController(BaseController):
    def __init__(self):
        super().__init__('mpc_controller')
        self.last_force = 0.0
        
        # --- System parameters ---
        M = 0.5      # cart mass (kg)
        m = 0.1      # pendulum mass (kg)
        L = 0.15     # pendulum length to centre of mass (m)
        g = 9.81     # gravity (m/s²)
        self.dt = 0.01    # timestep
        self.N = 20  # prediction horizon

        # --- Cost weights ---
        # [theta, theta_dot, x, x_dot]
        self.Q = np.diag([100.0, 1.0, 10.0, 1.0])
        self.R = np.array([[0.1]])

        # --- Continuous time matrices ---
        A_cont = np.array([
            [0,                     1, 0, 0],
            [(M + m) * g / (M * L), 0, 0, 0],
            [0,                     0, 0, 1],
            [-m * g / M,            0, 0, 0]
        ])

        B_cont = np.array([
            [0],
            [-1 / (M * L)],
            [0],
            [1 / M]
        ])

        # --- IMPROVEMENT 1: Exact Discretization ---
        # This converts continuous physics to exact discrete physics for dt=0.01
        C_dummy = np.eye(4)
        D_dummy = np.zeros((4, 1))
        self.A, self.B, _, _, _ = cont2discrete(
            (A_cont, B_cont, C_dummy, D_dummy), self.dt, method='zoh')

       
        self.x0_param = cp.Parameter(4)

        self.x_var = cp.Variable((4, self.N + 1))
        self.u_var = cp.Variable((1, self.N))

        cost = 0
        constraints = []

        # Initial condition constraint uses the Parameter
        constraints.append(self.x_var[:, 0] == self.x0_param)

        for k in range(self.N):
            cost += cp.quad_form(self.x_var[:, k], self.Q)
            cost += cp.quad_form(self.u_var[:, k], self.R)

            constraints.append(
                self.x_var[:, k + 1] == self.A @ self.x_var[:, k] + self.B @ self.u_var[:, k])

            # Input constraints
            constraints.append(self.u_var[:, k] <= 50.0)
            constraints.append(self.u_var[:, k] >= -50.0)

        # Terminal cost
        cost += cp.quad_form(self.x_var[:, self.N], self.Q)

        # State constraints
       # constraints.append(self.x_var[2, :] <= 1.2)
        # constraints.append(self.x_var[2, :] >= -0.9)

        # Build the problem ONCE in init
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def compute_control(self, state):

        if None in [state.pendulum_angle, state.pendulum_velocity, state.cart_position, state.cart_velocity]:
            self.get_logger().warn('Waiting for valid sensor data from Gazebo...',
                                   throttle_duration_sec=2.0)
            return 0.0

        self.get_logger().info(
            f'Angle: {state.pendulum_angle:.6f} | '
            f'Cart: {state.cart_position:.6f} | '
            f'Force: {self.last_force:.6f}',
            throttle_duration_sec=2.0)

        # Pack current state into a vector
        x0_current = np.array([
            float(state.pendulum_angle),
            float(state.pendulum_velocity),
            float(state.cart_position),
            float(state.cart_velocity)
        ], dtype=np.float64)

        if np.any(np.isnan(x0_current)) or np.any(np.isinf(x0_current)):
            self.get_logger().warn('Physics glitch (NaN/Inf) detected. Skipping cycle.',
                                   throttle_duration_sec=2.0)
            return 0.0

        self.x0_param.value = x0_current

        if self.x0_param.value is None:
            self.get_logger().error(
                'CRITICAL: CVXPY silently rejected the parameter assignment due to a shape/type mismatch.')
            return 0.0

        try:
            # warm_start=True makes it even faster by remembering the last solution
            self.problem.solve(solver=cp.OSQP, warm_start=True, max_iter=10000)

            if self.problem.status in ['optimal', 'optimal_inaccurate', 'user_limit']:
                # Extract the value safely
                u_val = self.u_var.value

                # Check to ensure CVXPY actually returned a value to satisfy the linter
                if u_val is not None:
                    # Use .item() to safely grab the first float regardless of 1D/2D shape
                    # Or use float(u_val[0]) if you strictly defined it as a 1D array
                    force = float(np.array(u_val).flatten()[0])
                    self.last_force = force

                    
                    force = max(min(force, 50.0), -50.0)
                    return force
                else:
                    return 0.0
            else:
                self.get_logger().warn(
                    f'MPC solver status: {self.problem.status}')
                return 0.0

        except Exception as e:
            self.get_logger().error(f'MPC solver failed: {str(e)}')
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    controller = MPCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
