"""
PID Controller FOR THE INVERTED PENDULUM...

ONE-TO-ONE mapping of standalone PID simulation (tesr.py) into the project's BaseController framework....

"""

import rclpy
import math
from control_base.base_controller import BaseController


class PIDController:
    """
    replica of the PIDController class from tesr.py.
    Same compute() method, same PID math.
    """

    def __init__(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.previous_error = 0.0

    def compute(self, theta_current: float, dt: float) -> float:
        # 0 is target andle (the upright stabilization)...
        theta_target = 0.0
        error = theta_target - theta_current

        # the P term...
        P = self.Kp * error

        # I term...
        self.integral_error += error * dt
        self.integral_error = max(-10.0, min(10.0, self.integral_error))
        I = self.Ki * self.integral_error

        # D term..
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative

        self.previous_error = error
        return P + I + D


class PIDControllerNode(BaseController):
    

    def __init__(self):
        super().__init__('pid_controller')

        # Declaring ROS parameters (matching initial tesr.py defaults)...
        self.declare_parameter('Kp', 50.0)
        self.declare_parameter('Ki', 10.0)
        self.declare_parameter('Kd', 5.0)
        self.declare_parameter('max_effort', 100.0)

        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value
        self.max_effort = self.get_parameter('max_effort').value

        # dt = 0.01 because the BaseController timer runs at 100Hz...
        self.dt = 0.01

        # Instantiating the PID... 
        self.pid = PIDController(Kp, Ki, Kd)

        self.get_logger().info(
            f'PID Controller started — Kp={Kp}, Ki={Ki}, Kd={Kd}'
        )

    def compute_control(self, state):
        
        self.pid.Kp = self.get_parameter('Kp').value
        self.pid.Ki = self.get_parameter('Ki').value
        self.pid.Kd = self.get_parameter('Kd').value

        # The core PID call...
        u = self.pid.compute(state.pendulum_angle, self.dt)

        # Clamp effort...
        u = max(-self.max_effort, min(self.max_effort, u))

        # Log once per second...
        self.get_logger().info(
            f'θ={math.degrees(state.pendulum_angle):+7.2f}°  '
            f'cart={state.cart_position:+6.3f}m  '
            f'u={u:+8.2f}N',
            throttle_duration_sec=1.0
        )

        return u


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
