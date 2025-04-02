#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
from controller_tutorials.control_apps import PID
from rcl_interfaces.msg import SetParametersResult

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class TurtleDualPIDController(Node):
    def __init__(self):
        super().__init__('turtle_dual_pid_controller')
        
        # --- Linear PID parameters ---
        self.declare_parameter('P_linear', 1.0)
        self.declare_parameter('I_linear', 0.0)
        self.declare_parameter('D_linear', 0.0)
        self.declare_parameter('max_linear', 5.0)
        self.declare_parameter('min_linear', -5.0)
        self.declare_parameter('distance_tolerance', 0.1)
        
        P_linear = self.get_parameter('P_linear').value
        I_linear = self.get_parameter('I_linear').value
        D_linear = self.get_parameter('D_linear').value
        max_linear = self.get_parameter('max_linear').value
        min_linear = self.get_parameter('min_linear').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # --- Angular PID parameters ---
        self.declare_parameter('P_angular', 5.0)
        self.declare_parameter('I_angular', 0.0)
        self.declare_parameter('D_angular', 0.0)
        self.declare_parameter('max_angular', 5.0)
        self.declare_parameter('min_angular', -5.0)
        self.declare_parameter('angular_tolerance', 0.01)
        
        P_angular = self.get_parameter('P_angular').value
        I_angular = self.get_parameter('I_angular').value
        D_angular = self.get_parameter('D_angular').value
        max_angular = self.get_parameter('max_angular').value
        min_angular = self.get_parameter('min_angular').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        
        # Create PID controllers for linear and angular control
        self.linear_pid = PID()
        self.linear_pid.P = P_linear
        self.linear_pid.I = I_linear
        self.linear_pid.D = D_linear
        self.linear_pid.max_state = max_linear
        self.linear_pid.min_state = min_linear
        
        self.angular_pid = PID()
        self.angular_pid.P = P_angular
        self.angular_pid.I = I_angular
        self.angular_pid.D = D_angular
        self.angular_pid.max_state = max_angular
        self.angular_pid.min_state = min_angular
        
        # Goal pose (turtlesim/Pose)
        # 외부에서 goal_pose가 발행되기 전까지는 초기 터틀 위치를 goal_pose로 설정
        self.goal_pose = Pose()
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        self.goal_pose.theta = 0.0
        self.initial_goal_set = False
        
        # Subscriber: 현재 터틀의 pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        
        # Subscriber: 외부 goal_pose (turtlesim/Pose)
        self.goal_subscriber = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_callback,
            10)
        
        # Publisher: cmd_vel 토픽으로 제어 명령 발행
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Publisher: 거리 오차를 Float64 메시지로 발행
        self.error_publisher = self.create_publisher(Float64, 'error', 10)
        
        # 동적 파라미터 재설정 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'P_linear':
                self.linear_pid.P = param.value
                self.get_logger().info(f"Updated linear PID P: {param.value}")
            elif param.name == 'I_linear':
                self.linear_pid.I = param.value
                self.get_logger().info(f"Updated linear PID I: {param.value}")
            elif param.name == 'D_linear':
                self.linear_pid.D = param.value
                self.get_logger().info(f"Updated linear PID D: {param.value}")
            elif param.name == 'max_linear':
                self.linear_pid.max_state = param.value
                self.get_logger().info(f"Updated linear PID max_linear: {param.value}")
            elif param.name == 'min_linear':
                self.linear_pid.min_state = param.value
                self.get_logger().info(f"Updated linear PID min_linear: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                self.get_logger().info(f"Updated distance_tolerance: {param.value}")
            elif param.name == 'P_angular':
                self.angular_pid.P = param.value
                self.get_logger().info(f"Updated angular PID P: {param.value}")
            elif param.name == 'I_angular':
                self.angular_pid.I = param.value
                self.get_logger().info(f"Updated angular PID I: {param.value}")
            elif param.name == 'D_angular':
                self.angular_pid.D = param.value
                self.get_logger().info(f"Updated angular PID D: {param.value}")
            elif param.name == 'max_angular':
                self.angular_pid.max_state = param.value
                self.get_logger().info(f"Updated angular PID max_angular: {param.value}")
            elif param.name == 'min_angular':
                self.angular_pid.min_state = param.value
                self.get_logger().info(f"Updated angular PID min_angular: {param.value}")
            elif param.name == 'angular_tolerance':
                self.angular_tolerance = param.value
                self.get_logger().info(f"Updated angular_tolerance: {param.value}")
        return SetParametersResult(successful=True)
    
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(
            f"Received new goal_pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
    
    def pose_callback(self, msg):
        current_pose = msg
        
        # 만약 아직 외부 goal_pose가 설정되지 않았다면,
        # 현재 터틀의 pose를 goal_pose로 복사하여 터틀이 움직이지 않도록 함.
        if not self.initial_goal_set:
            self.goal_pose = msg
            self.initial_goal_set = True
            self.get_logger().info("Initial goal_pose set to current turtle pose.")
        
        twist_msg = Twist()
        
        # 현재 위치에서 goal_pose까지의 벡터 계산
        dx = self.goal_pose.x - current_pose.x
        dy = self.goal_pose.y - current_pose.y
        # 로봇의 현재 heading을 기준으로 signed distance error 계산
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        
        # 에러 토픽에 거리 오차 발행
        error_msg = Float64()
        error_msg.data = distance_error
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f"[Move to Goal] Distance error: {distance_error:.2f}")
        
        # distance tolerance 적용: 오차가 tolerance 이내면 제어 출력 0
        if abs(distance_error) < self.distance_tolerance:
            twist_msg.linear.x = 0.0
        else:
            linear_correction = self.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
        
        # 이동 중 목표 방향 보정을 위한 각 PID 제어
        desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                    self.goal_pose.x - current_pose.x)
        angle_error = normalize_angle(desired_heading - current_pose.theta)
        
        # angular tolerance 적용
        if abs(angle_error) < self.angular_tolerance:
            twist_msg.angular.z = 0.0
        else:
            angular_correction = self.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(
            f"Current Pose: (x={current_pose.x:.2f}, y={current_pose.y:.2f}, theta={current_pose.theta:.2f}), " +
            f"Goal Pose: (x={self.goal_pose.x:.2f}, y={self.goal_pose.y:.2f}, theta={self.goal_pose.theta:.2f}), " +
            f"Angle error: {angle_error:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDualPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
