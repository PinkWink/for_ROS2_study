#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import multiprocessing
import math
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

class TurtleRotator(Node):
    def __init__(self, domain_id: int, target_theta_deg: float):
        super().__init__(f'turtle_rotator_{domain_id}')
        self.domain_id = domain_id
        # 입력받은 각도(도)를 라디안으로 변환
        self.target_theta = math.radians(target_theta_deg)
        # rotate_absolute 액션 클라이언트 생성
        self._action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')

    def send_goal(self):
        # 액션 서버가 준비될 때까지 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("RotateAbsolute action server not available")
            return

        # 목표 각도를 포함하는 Goal 메시지 생성
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = self.target_theta
        self.get_logger().info(
            f"[Domain {self.domain_id}] Sending goal to rotate to {self.target_theta:.2f} radians "
            f"({math.degrees(self.target_theta):.1f} degrees)"
        )

        # 액션 서버에 목표 전송
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        # 결과를 기다림
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"[Domain {self.domain_id}] RotateAbsolute action result: {result}")

def rotate_process(domain_id: int, target_theta_deg: float):
    # 각 프로세스에서 해당 도메인으로 rclpy 초기화
    rclpy.init(args=[], domain_id=domain_id)
    node = TurtleRotator(domain_id, target_theta_deg)
    node.send_goal()
    node.destroy_node()
    rclpy.shutdown()

def main():
    # Domain 1: turtle1을 90도로 회전
    p1 = multiprocessing.Process(target=rotate_process, args=(1, 90.0))
    # Domain 2: turtle1을 270도로 회전
    p2 = multiprocessing.Process(target=rotate_process, args=(2, 270.0))
    
    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
