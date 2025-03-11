#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import multiprocessing

class TurtleController(Node):
    def __init__(self, domain_id: int, angular_speed: float):
        super().__init__(f'turtle_controller_{domain_id}')
        # turtlesim의 터틀을 제어하기 위한 cmd_vel 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.angular_speed = angular_speed
        self.domain_id = domain_id
        # 1초마다 timer_callback 호출
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        # 각속도는 지정된 값(양수: 반시계, 음수: 시계)로 설정
        msg.linear.x = 1.0
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        direction = "Counterclockwise" if self.angular_speed > 0 else "Clockwise"
        self.get_logger().info(
            f"[Domain {self.domain_id}] Published: {direction} rotation with angular.z = {msg.angular.z}"
        )

def turtle_process(domain_id: int, angular_speed: float):
    # 각 프로세스에서 해당 도메인으로 rclpy 초기화
    rclpy.init(args=[], domain_id=domain_id)
    node = TurtleController(domain_id, angular_speed)
    try:
        # rclpy.spin으로 timer_callback이 계속 호출되도록 함
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def main():
    # domain 1: 시계방향 회전 (angular_speed 음수)
    p1 = multiprocessing.Process(target=turtle_process, args=(1, -1.0))
    # domain 2: 반시계방향 회전 (angular_speed 양수)
    p2 = multiprocessing.Process(target=turtle_process, args=(2, 1.0))
    
    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
