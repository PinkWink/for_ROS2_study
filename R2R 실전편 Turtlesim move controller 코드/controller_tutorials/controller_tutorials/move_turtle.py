import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
import math
from controller_tutorials.control_apps import PID

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class TurtleGoalController(Node):
    def __init__(self):
        super().__init__('turtle_goal_controller')
        
        # ROS2 파라미터 선언 (제어 속도, 허용 오차 및 PID 파라미터)
        self.declare_parameter('angle_tolerance', 0.01)
        self.declare_parameter('distance_tolerance', 0.01)
        
        # Angular PID 파라미터
        self.declare_parameter('angular_P', 1.0)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 2.0)
        self.declare_parameter('angular_min_state', -2.0)
        
        # Linear PID 파라미터
        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 2.0)  # 최대 선형 속도는 2
        self.declare_parameter('linear_min_state', -2.0)
        
        # 초기 파라미터 값 가져오기
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Angular PID 초기화
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
        
        # Linear PID 초기화
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

        # 상태 변수 설정
        # 상태: "idle", "rotate_to_goal", "move_to_goal", "rotate_to_final", "goal_reached"
        self.state = "idle"
        
        # goal_pose 토픽에서 수신할 목표 위치 및 최종 자세 (초기값 None)
        self.goal_pose = None
        
        # Subscriber 및 Publisher 생성
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
        
        # 파라미터 동적 재구성을 위한 콜백 등록
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
        # goal_pose 토픽에서 수신한 목표 위치 및 최종 자세 업데이트 후 상태 전환
        self.goal_pose = msg
        self.state = "rotate_to_goal"
        self.get_logger().info(
            f"Received new goal pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
    
    def pose_callback(self, msg):
        # 목표가 없으면 제어 동작을 수행하지 않음
        if self.goal_pose is None:
            return
        self.control(msg)
    
    def control(self, current_pose):
        twist_msg = Twist()
        error_msg = Float64()
        
        if self.state == "rotate_to_goal":
            # 현재 위치에서 목표 위치까지의 방향(heading) 계산
            desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                         self.goal_pose.x - current_pose.x)
            # 헤딩 에러 계산 (정규화)
            error_angle = normalize_angle(desired_heading - current_pose.theta)
            error_msg.data = error_angle
            self.error_publisher.publish(error_msg)
            self.get_logger().info(f"[Rotate to Goal] Heading error: {error_angle:.2f}")
            
            if abs(error_angle) > self.angle_tolerance:
                # Angular PID를 사용하여 제어 출력 계산
                angular_correction = self.angular_pid.update(error_angle)
                twist_msg.angular.z = angular_correction
                twist_msg.linear.x = 0.0
            else:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.state = "move_to_goal"
                self.get_logger().info("Heading aligned. Switching to move_to_goal state.")
                
        elif self.state == "move_to_goal":
            # Calculate vector from current position to goal
            dx = self.goal_pose.x - current_pose.x
            dy = self.goal_pose.y - current_pose.y
            # Compute signed distance error along the robot's current heading
            distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
            error_msg.data = distance_error
            self.error_publisher.publish(error_msg)
            self.get_logger().info(f"[Move to Goal] Distance error: {distance_error:.2f}")
            
            if abs(distance_error) > self.distance_tolerance:
                # Use linear PID to compute forward control output
                linear_correction = self.linear_pid.update(distance_error)
                twist_msg.linear.x = linear_correction
                
                # Also, correct heading during movement
                desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                             self.goal_pose.x - current_pose.x)
                angle_error = normalize_angle(desired_heading - current_pose.theta)
                angular_correction = self.angular_pid.update(angle_error)
                twist_msg.angular.z = angular_correction
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.state = "rotate_to_final"
                self.get_logger().info("Position reached. Switching to rotate_to_final state.")
                
        elif self.state == "rotate_to_final":
            # 목표 최종 자세에 맞추기 위해 회전 (최종 theta와의 오차 계산)
            final_error = normalize_angle(self.goal_pose.theta - current_pose.theta)
            error_msg.data = final_error
            self.error_publisher.publish(error_msg)
            self.get_logger().info(f"[Rotate to Final] Orientation error: {final_error:.2f}")
            
            if abs(final_error) > self.angle_tolerance:
                # Angular PID를 사용하여 최종 회전 제어 출력 계산
                angular_correction = self.angular_pid.update(final_error)
                twist_msg.angular.z = angular_correction
                twist_msg.linear.x = 0.0
            else:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.state = "goal_reached"
                self.get_logger().info("Final orientation reached. Goal achieved.")
                
        elif self.state == "goal_reached":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            # 필요 시 상태를 "idle"로 전환하거나 새로운 목표를 기다림
        
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