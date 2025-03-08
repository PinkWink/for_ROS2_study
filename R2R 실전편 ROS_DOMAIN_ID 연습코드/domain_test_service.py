#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import multiprocessing
from turtlesim.srv import TeleportAbsolute

class TurtleTeleporter(Node):
    def __init__(self, domain_id: int, target_x: float):
        super().__init__(f'turtle_teleporter_{domain_id}')
        self.domain_id = domain_id
        self.target_x = target_x
        # teleport_absolute 서비스 클라이언트 생성
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

    def call_teleport(self):
        # 서비스가 준비될 때까지 대기
        if not self.teleport_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Teleport service not available")
            return
        
        # 요청 메시지 구성 (y, theta는 기본값 사용)
        request = TeleportAbsolute.Request()
        request.x = self.target_x
        request.y = 5.544  # 기본 y 좌표 (필요시 조정)
        request.theta = 0.0
        self.get_logger().info(f"[Domain {self.domain_id}] Teleporting turtle1 to x = {self.target_x}")
        # 서비스 호출 및 결과 대기
        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
            self.get_logger().info(f"[Domain {self.domain_id}] Teleport service call successful.")
        except Exception as e:
            self.get_logger().error(f"Teleport service call failed: {e}")

def teleporter_process(domain_id: int, target_x: float):
    # 각 프로세스에서 해당 도메인으로 rclpy 초기화
    rclpy.init(args=[], domain_id=domain_id)
    node = TurtleTeleporter(domain_id, target_x)
    node.call_teleport()
    node.destroy_node()
    rclpy.shutdown()

def main():
    # domain 1: x 좌표를 2로 teleport
    p1 = multiprocessing.Process(target=teleporter_process, args=(1, 2.0))
    # domain 2: x 좌표를 8로 teleport
    p2 = multiprocessing.Process(target=teleporter_process, args=(2, 8.0))
    
    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
