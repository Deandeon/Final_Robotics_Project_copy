import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from control_base.robot_state import RobotState


class BaseController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.state = RobotState()

        # from the sensors
        self.create_subscription(Imu, '/pendulum_imu', self.imu_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(
            JointState, '/cart_joint_states', self.cart_callback, 10)

        # Send force commands to the cart
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/cart_effort_controller/commands', 10)

        
        self.create_timer(0.01, self.control_loop)

    # angular velocity from IMU
    def imu_callback(self, msg):
        self.state.pendulum_velocity = msg.angular_velocity.y


    # angle from pendulum joint 
    def joint_callback(self, msg):
        
       if 'rail_to_cart' in msg.name:
           idx = msg.name.index('rail_to_cart')
           if len(msg.position) > idx:
                self.state.cart_position = msg.position[idx]
           if len(msg.velocity) > idx:
                self.state.cart_velocity = msg.velocity[idx]
       
       if 'pendulum_joint' in msg.name:
            idx = msg.name.index('pendulum_joint')
            if len(msg.position) > idx:
                self.state.pendulum_angle = msg.position[idx]
            if len(msg.velocity) > idx:
                self.state.pendulum_velocity = msg.velocity[idx]
            

    # position and velocity from  the cart joint
    def cart_callback(self, msg):
        
         if 'rail_to_cart' in msg.name:
            idx = msg.name.index('rail_to_cart')
            self.state.cart_position = msg.position[idx]
            self.state.cart_velocity = msg.velocity[idx]

    def control_loop(self):
        force = self.compute_control(self.state)

        msg = Float64MultiArray()
        msg.data = [force]
        self.effort_pub.publish(msg)

    def compute_control(self, state):
      
        return 0.0
