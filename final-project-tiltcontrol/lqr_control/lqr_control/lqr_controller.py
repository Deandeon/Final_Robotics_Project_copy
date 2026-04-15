import rclpy
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import cont2discrete
from control_base.base_controller import BaseController


class LQRController(BaseController):
    """LQR controller for inverted pendulum on cart."""

    def __init__(self):
        """Initialise LQR controller with system model and gain computation."""
        super().__init__('lqr_controller')

        # --- System parameters (must match URDF / MPC controller) ---
        M = 0.5      # cart mass (kg)
        m = 0.1      # pendulum mass (kg)
        L = 0.15     # pendulum length to centre of mass (m)
        g = 9.81     # gravity (m/s^2)
        self.dt = 0.01  # control timestep (s)

        # --- Force saturation limit ---
        self.max_force = 100.00 # N

        # --- Continuous-time linearised state-space matrices ---
        # State vector: [theta, theta_dot, x, x_dot]
        # theta = pendulum angle (rad, 0 = upright)
        # x     = cart position (m)
        A_cont = np.array([
            [0,                     1, 0, 0],
            [(M + m) * g / (M * L), 0, 0, 0],
            [0,                     0, 0, 1],
            [-m * g / M,            0, 0, 0]
        ])

        B_cont = np.array([
            [0],
            [-1.0 / (M * L)],
            [0],
            [1.0 / M]
        ])

        # --- LQR Cost Weights ---
        # Q penalises state error:  [theta, theta_dot, x, x_dot]
        # R penalises control effort
        # Increase Q[0,0] to prioritise angle stability over cart position.
        Q = np.diag([500.0, 10.0, 1.0, 5.0])
        R = np.array([[0.01]]) 

        # --- Compute LQR gain via continuous-time Algebraic Riccati Equation ---
        # Solve: A^T P + P A - P B R^-1 B^T P + Q = 0
        P = solve_continuous_are(A_cont, B_cont, Q, R)
        R_inv = np.linalg.inv(R)
        self.K = R_inv @ B_cont.T @ P  # shape (1, 4)

        # --- Discretise for reference (used for logging only) ---
        C_dummy = np.eye(4)
        D_dummy = np.zeros((4, 1))
        A_d, B_d, _, _, _ = cont2discrete(
            (A_cont, B_cont, C_dummy, D_dummy), self.dt, method='zoh')
        self.A_d = A_d
        self.B_d = B_d

        self.get_logger().info(
            f'LQR gain K = {np.round(self.K, 4)}'
        )

    def compute_control(self, state):
        """Compute LQR control force given current robot state."""
        if None in [
            state.pendulum_angle,
            state.pendulum_velocity,
            state.cart_position,
            state.cart_velocity
        ]:
            return 0.0

        # Pack state: [theta, theta_dot, x, x_dot]
        x = np.array([
            state.pendulum_angle,
            state.pendulum_velocity,
            state.cart_position,
            state.cart_velocity
        ], dtype=float)

        # LQR control law: u = -K * x
        # Negative sign: controller pushes back against deviation
        force = float(-self.K @ x)

        # Saturate output
        force = max(min(force, self.max_force), -self.max_force)
        return force


def main(args=None):
    """Entry point for LQR controller node."""
    rclpy.init(args=args)
    controller = LQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()