#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from controller_tutorials.control_apps import PID
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64

class TurtlePIDController(Node):
    def __init__(self):
        super().__init__('turtle_pid_controller')
        
        # Declare ROS2 parameters (PID parameters and tolerance)
        self.declare_parameter('P', 1.0)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 0.0)
        self.declare_parameter('max_state', 5.0)
        self.declare_parameter('min_state', -5.0)
        self.declare_parameter('tolerance', 0.01)
        
        # Get initial parameter values for PID and tolerance
        P = self.get_parameter('P').value
        I = self.get_parameter('I').value
        D = self.get_parameter('D').value
        max_state = self.get_parameter('max_state').value
        min_state = self.get_parameter('min_state').value
        self.tolerance = self.get_parameter('tolerance').value

        # Create PID instance and assign parameters
        self.pid = PID()
        self.pid.P = P
        self.pid.I = I
        self.pid.D = D
        self.pid.max_state = max_state
        self.pid.min_state = min_state

        # Target angle is received from the goal_theta topic (initially 0.0)
        self.target_theta = 0.0

        # Subscribe to turtlesim pose and publish to cmd_vel
        self.pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Subscribe to goal_theta topic (std_msgs/Float64)
        self.goal_subscriber = self.create_subscription(
            Float64,
            'goal_theta',
            self.goal_callback,
            10)
        
        # Publisher for error topic (std_msgs/Float64)
        self.error_publisher = self.create_publisher(Float64, 'error', 10)
        
        # Register parameter callback for dynamic reconfiguration (PID parameters and tolerance)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'P':
                self.pid.P = param.value
                self.get_logger().info(f"Updated PID P: {param.value}")
            elif param.name == 'I':
                self.pid.I = param.value
                self.get_logger().info(f"Updated PID I: {param.value}")
            elif param.name == 'D':
                self.pid.D = param.value
                self.get_logger().info(f"Updated PID D: {param.value}")
            elif param.name == 'max_state':
                self.pid.max_state = param.value
                self.get_logger().info(f"Updated PID max_state: {param.value}")
            elif param.name == 'min_state':
                self.pid.min_state = param.value
                self.get_logger().info(f"Updated PID min_state: {param.value}")
            elif param.name == 'tolerance':
                self.tolerance = param.value
                self.get_logger().info(f"Updated tolerance: {param.value}")
        return SetParametersResult(successful=True)

    def goal_callback(self, msg):
        # Update target angle from the goal_theta topic
        self.target_theta = msg.data
        self.get_logger().info(f"Received new goal_theta: {self.target_theta:.2f}")

    def pose_callback(self, msg):
        # Calculate error between current angle and target angle (normalized to -pi ~ pi)
        error = self.target_theta - msg.theta
        error = math.atan2(math.sin(error), math.cos(error))
        
        # Publish error on the error topic
        error_msg = Float64()
        error_msg.data = error
        self.error_publisher.publish(error_msg)
        
        twist_msg = Twist()
        # If error is within tolerance, stop the controller (output 0)
        if abs(error) < self.tolerance:
            twist_msg.angular.z = 0.0
            self.get_logger().info("Error within tolerance. Controller stopped.")
        else:
            # Compute control output (angular velocity) using PID controller
            angular_correction = self.pid.update(error)
            twist_msg.angular.z = angular_correction
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        
        self.get_logger().info(
            f"Current theta: {msg.theta:.2f}, Target theta: {self.target_theta:.2f}, Error: {error:.2f}, "
            f"Control Output: {twist_msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()