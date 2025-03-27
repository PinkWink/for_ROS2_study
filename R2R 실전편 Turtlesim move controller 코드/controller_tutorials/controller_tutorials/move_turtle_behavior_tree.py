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

# Behavior Tree execution status
class BTStatus:
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

# Abstract Behavior Tree Node (returns a (Twist, status) tuple)
class BTNode:
    def tick(self, current_pose):
        raise NotImplementedError("tick() must be implemented by subclasses")

# Sequence Node: Executes its child nodes sequentially
class SequenceNode(BTNode):
    def __init__(self, children):
        self.children = children
        self.current = 0

    def tick(self, current_pose):
        while self.current < len(self.children):
            status, twist = self.children[self.current].tick(current_pose)
            if status == BTStatus.RUNNING:
                return BTStatus.RUNNING, twist
            elif status == BTStatus.FAILURE:
                self.current = 0
                return BTStatus.FAILURE, twist
            elif status == BTStatus.SUCCESS:
                self.current += 1
        self.current = 0
        return BTStatus.SUCCESS, twist

# RotateToGoalNode: Rotate to face the goal position
class RotateToGoalNode(BTNode):
    def __init__(self, controller):
        self.controller = controller

    def tick(self, current_pose):
        twist_msg = Twist()
        desired_heading = math.atan2(self.controller.goal_pose.y - current_pose.y,
                                        self.controller.goal_pose.x - current_pose.x)
        error_angle = normalize_angle(desired_heading - current_pose.theta)
        error_msg = Float64()
        error_msg.data = error_angle
        self.controller.error_publisher.publish(error_msg)
        self.controller.publish_bt_state("RotateToGoal")
        self.controller.get_logger().info(f"[RotateToGoal] Heading error: {error_angle:.2f}")
        
        if abs(error_angle) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return BTStatus.RUNNING, twist_msg
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("RotateToGoal: SUCCESS")
            return BTStatus.SUCCESS, twist_msg

# MoveToGoalNode: Move forward toward the goal while correcting heading
class MoveToGoalNode(BTNode):
    def __init__(self, controller):
        self.controller = controller

    def tick(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y
        # Signed distance error along robot's current heading
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        error_msg = Float64()
        error_msg.data = distance_error
        self.controller.error_publisher.publish(error_msg)
        self.controller.publish_bt_state("MoveToGoal")
        self.controller.get_logger().info(f"[MoveToGoal] Distance error: {distance_error:.2f}")
        
        if abs(distance_error) > self.controller.distance_tolerance:
            linear_correction = self.controller.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            desired_heading = math.atan2(self.controller.goal_pose.y - current_pose.y,
                                            self.controller.goal_pose.x - current_pose.x)
            angle_error = normalize_angle(desired_heading - current_pose.theta)
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            return BTStatus.RUNNING, twist_msg
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.controller.get_logger().info("MoveToGoal: SUCCESS")
            return BTStatus.SUCCESS, twist_msg

# RotateToFinalNode: Rotate in place to match the final orientation
class RotateToFinalNode(BTNode):
    def __init__(self, controller):
        self.controller = controller

    def tick(self, current_pose):
        twist_msg = Twist()
        final_error = normalize_angle(self.controller.goal_pose.theta - current_pose.theta)
        error_msg = Float64()
        error_msg.data = final_error
        self.controller.error_publisher.publish(error_msg)
        self.controller.publish_bt_state("RotateToFinal")
        self.controller.get_logger().info(f"[RotateToFinal] Orientation error: {final_error:.2f}")
        
        if abs(final_error) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return BTStatus.RUNNING, twist_msg
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("RotateToFinal: SUCCESS")
            return BTStatus.SUCCESS, twist_msg

# Main controller node that manages the Behavior Tree
class TurtleGoalController(Node):
    def __init__(self):
        super().__init__('turtle_goal_controller')
        
        # Declare ROS2 parameters for tolerances and PID parameters
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('distance_tolerance', 0.1)
        
        # Angular PID parameters
        self.declare_parameter('angular_P', 1.0)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 2.0)
        self.declare_parameter('angular_min_state', -2.0)
        
        # Linear PID parameters
        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 2.0)
        self.declare_parameter('linear_min_state', -2.0)
        
        # Get initial parameter values
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

        # Initialize goal pose and Behavior Tree root
        self.goal_pose = None
        self.bt_root = None
        
        # Subscribers and Publishers
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
        self.error_publisher = self.create_publisher(Float64, 'error', 10)
        self.state_publisher = self.create_publisher(String, 'bt_state', 10)
        # Separate publishers for angle and distance error
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        
        # Register parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def publish_bt_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.state_publisher.publish(msg)
    
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
        self.bt_root = SequenceNode([
            RotateToGoalNode(self),
            MoveToGoalNode(self),
            RotateToFinalNode(self)
        ])
        self.get_logger().info(
            f"Received new goal pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
    
    def pose_callback(self, msg):
        if self.goal_pose is None or self.bt_root is None:
            return
        status, twist_msg = self.bt_root.tick(msg)
        self.cmd_vel_publisher.publish(twist_msg)
        # When the behavior tree sequence is complete, stop further commands.
        if status == BTStatus.SUCCESS:
            self.get_logger().info("Goal achieved. Stopping behavior tree.")
            self.bt_root = None
            self.goal_pose = None

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
