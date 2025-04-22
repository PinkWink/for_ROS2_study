#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult
import math

from controller_tutorials.control_apps import PID


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Define possible state results (IDLE removed)
class StateResult:
    CONTINUE = 0
    COMPLETE = 1


# Abstract state class that returns a (Twist, status) tuple
class ControllerState:
    def __init__(self, controller):
        self.controller = controller

    def update(self, current_pose):
        raise NotImplementedError("update() must be implemented by subclasses")


# RotateToGoalState: turn to face the goal
class RotateToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        desired_heading = math.atan2(
            self.controller.goal_pose.y - current_pose.y,
            self.controller.goal_pose.x - current_pose.x)
        error_angle = normalize_angle(desired_heading - current_pose.theta)
        error_msg = Float64()
        error_msg.data = error_angle
        # Publish angle error and state
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[RotateToGoal] Heading error: {error_angle:.2f}")

        if abs(error_angle) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("Heading aligned. RotateToGoal complete.")
            return twist_msg, StateResult.COMPLETE


# MoveToGoalState: move forward toward the goal while correcting heading
class MoveToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        error_msg = Float64()
        error_msg.data = distance_error
        # Publish distance error and state
        self.controller.distance_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[MoveToGoal] Distance error: {distance_error:.2f}")

        if abs(distance_error) > self.controller.distance_tolerance:
            linear_correction = self.controller.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            desired_heading = math.atan2(
                self.controller.goal_pose.y - current_pose.y,
                self.controller.goal_pose.x - current_pose.x)
            angle_error = normalize_angle(desired_heading - current_pose.theta)
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.controller.get_logger().info("Position reached. MoveToGoal complete.")
            return twist_msg, StateResult.COMPLETE


# RotateToFinalState: rotate in place to match final orientation
class RotateToFinalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        final_error = normalize_angle(self.controller.goal_pose.theta - current_pose.theta)
        error_msg = Float64()
        error_msg.data = final_error
        # Publish final orientation error and state
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[RotateToFinal] Orientation error: {final_error:.2f}")

        if abs(final_error) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("Final orientation reached. RotateToFinal complete.")
            return twist_msg, StateResult.COMPLETE


# GoalReachedState: the terminal state when the goal has been achieved
class GoalReachedState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.controller.state_publisher.publish(String(data="GoalReached"))
        self.controller.get_logger().info("Goal reached!")
        return twist_msg, StateResult.COMPLETE


# StateTransitionManager encapsulates state transition rules
class StateTransitionManager:
    def __init__(self, controller):
        self.controller = controller

    def get_next_state(self, current_state, state_result):
        if state_result == StateResult.COMPLETE:
            if isinstance(current_state, RotateToGoalState):
                return MoveToGoalState(self.controller)
            elif isinstance(current_state, MoveToGoalState):
                return RotateToFinalState(self.controller)
            elif isinstance(current_state, RotateToFinalState):
                return GoalReachedState(self.controller)
            elif isinstance(current_state, GoalReachedState):
                # Terminal state, remain here
                return GoalReachedState(self.controller)
        elif state_result == StateResult.CONTINUE:
            return current_state
        return current_state


# Main controller node that manages state transitions
class TurtleGoalController(Node):
    def __init__(self):
        super().__init__('turtle_goal_controller')
        
        # Declare parameters for tolerances and PID
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('distance_tolerance', 0.1)

        self.declare_parameter('angular_P', 2.0)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 2.0)
        self.declare_parameter('angular_min_state', -2.0)

        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 2.0)
        self.declare_parameter('linear_min_state', -2.0)

        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # Initialize Angular PID
        angular_P = self.get_parameter('angular_P').value
        angular_I = self.get_parameter('angular_I').value
        angular_D = self.get_parameter('angular_D').value
        angular_max_state = self.get_parameter('angular_max_state').value
        angular_min_state = self.get_parameter('angular_min_state').value
        self.angular_pid = PID()
        self.angular_pid.P = angular_P
        self.angular_pid.I = angular_I
        self.angular_pid.D = angular_D
        self.angular_pid.max_state = angular_max_state
        self.angular_pid.min_state = angular_min_state

        # Initialize Linear PID
        linear_P = self.get_parameter('linear_P').value
        linear_I = self.get_parameter('linear_I').value
        linear_D = self.get_parameter('linear_D').value
        linear_max_state = self.get_parameter('linear_max_state').value
        linear_min_state = self.get_parameter('linear_min_state').value
        self.linear_pid = PID()
        self.linear_pid.P = linear_P
        self.linear_pid.I = linear_I
        self.linear_pid.D = linear_D
        self.linear_pid.max_state = linear_max_state
        self.linear_pid.min_state = linear_min_state

        self.state_instance = None
        self.goal_pose = None
        
        self.state_transition_manager = StateTransitionManager(self)
        
        self.pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.goal_pose_subscriber = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'angle_tolerance':
                self.angle_tolerance = param.value
                self.get_logger().info(f"Updated angle_tolerance: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                self.get_logger().info(f"Updated distance_tolerance: {param.value}")
            elif param.name == 'angular_P':
                self.angular_pid.P = param.value
                self.get_logger().info(f"Updated angular_PID P: {param.value}")
            elif param.name == 'angular_I':
                self.angular_pid.I = param.value
                self.get_logger().info(f"Updated angular_PID I: {param.value}")
            elif param.name == 'angular_D':
                self.angular_pid.D = param.value
                self.get_logger().info(f"Updated angular_PID D: {param.value}")
            elif param.name == 'angular_max_state':
                self.angular_pid.max_state = param.value
                self.get_logger().info(f"Updated angular_PID max_state: {param.value}")
            elif param.name == 'angular_min_state':
                self.angular_pid.min_state = param.value
                self.get_logger().info(f"Updated angular_PID min_state: {param.value}")
            elif param.name == 'linear_P':
                self.linear_pid.P = param.value
                self.get_logger().info(f"Updated linear_PID P: {param.value}")
            elif param.name == 'linear_I':
                self.linear_pid.I = param.value
                self.get_logger().info(f"Updated linear_PID I: {param.value}")
            elif param.name == 'linear_D':
                self.linear_pid.D = param.value
                self.get_logger().info(f"Updated linear_PID D: {param.value}")
            elif param.name == 'linear_max_state':
                self.linear_pid.max_state = param.value
                self.get_logger().info(f"Updated linear_PID max_state: {param.value}")
            elif param.name == 'linear_min_state':
                self.linear_pid.min_state = param.value
                self.get_logger().info(f"Updated linear_PID min_state: {param.value}")
        return SetParametersResult(successful=True)
    
    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        self.state_instance = RotateToGoalState(self)
        self.get_logger().info(
            f"Received new goal pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
    
    def pose_callback(self, msg):
        if self.goal_pose is None or self.state_instance is None:
            return
        twist_msg, status = self.state_instance.update(msg)
        self.state_instance = self.state_transition_manager.get_next_state(self.state_instance, status)
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
