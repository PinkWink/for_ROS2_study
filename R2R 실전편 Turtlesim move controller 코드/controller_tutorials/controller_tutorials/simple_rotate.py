import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
import math

class TurtleConstantAngularController(Node):
    def __init__(self):
        super().__init__('turtle_constant_angular_controller')
        # ROS2 파라미터 선언 (기본값 설정)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('tolerance', 0.01)

        # 파라미터 값 가져오기
        self.angular_speed = self.get_parameter('angular_speed').value
        self.tolerance = self.get_parameter('tolerance').value

        # 파라미터 동적 재구성을 위한 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 현재 회전각과 목표 회전각 초기화
        self.current_theta = 0.0
        self.target_theta = 0.0  # goal_theta 토픽에서 업데이트됨

        # turtlesim pose 토픽 구독
        self.pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10
        )
        # goal_theta 토픽 구독 (std_msgs/Float64)
        self.goal_subscriber = self.create_subscription(
            Float64,
            'goal_theta',
            self.goal_callback,
            10
        )
        # cmd_vel 토픽 발행
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # error 토픽 발행 (계산된 오차를 발행)
        self.error_publisher = self.create_publisher(Float64, 'error', 10)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'angular_speed':
                self.angular_speed = param.value
                self.get_logger().info(f"Updated angular_speed: {self.angular_speed}")
            elif param.name == 'tolerance':
                self.tolerance = param.value
                self.get_logger().info(f"Updated tolerance: {self.tolerance}")
        result = SetParametersResult()
        result.successful = True
        return result

    def pose_callback(self, msg):
        self.current_theta = msg.theta
        self.control()

    def goal_callback(self, msg):
        self.target_theta = msg.data
        self.get_logger().info(f"Received new goal_theta: {self.target_theta:.2f}")

    def control(self):
        # 현재 theta와 목표 theta 간의 오차 계산 및 -pi ~ pi 범위로 정규화
        error = self.target_theta - self.current_theta
        error = math.atan2(math.sin(error), math.cos(error))
        
        # error 토픽 발행
        error_msg = Float64()
        error_msg.data = error
        self.error_publisher.publish(error_msg)

        twist_msg = Twist()
        # 오차가 허용 범위보다 크면, 일정한 각속도로 회전
        if abs(error) > self.tolerance:
            twist_msg.angular.z = self.angular_speed if error > 0 else -self.angular_speed
        else:
            twist_msg.angular.z = 0.0  # 오차가 충분히 작으면 정지
        twist_msg.linear.x = 0.0

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(
            f"Current theta: {self.current_theta:.2f}, Goal theta: {self.target_theta:.2f}, Error: {error:.2f}, angular.z: {twist_msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtleConstantAngularController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()