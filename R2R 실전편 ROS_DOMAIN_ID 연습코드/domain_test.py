#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import multiprocessing

class DomainPublisher(Node):
    def __init__(self, domain_id: int, start_value: int, increment: int):
        super().__init__(f'domain_publisher_{domain_id}')
        self.publisher_ = self.create_publisher(Int32, 'id_test', 10)
        self.value = start_value
        self.increment = increment
        self.domain_id = domain_id
        # 1초마다 timer_callback 호출
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.value
        self.publisher_.publish(msg)
        self.get_logger().info(f"[Domain {self.domain_id}] Published: {msg.data}")
        self.value += self.increment

def publisher_process(domain_id: int, start_value: int, increment: int):
    # 각 프로세스에서 해당 도메인으로 rclpy 초기화
    rclpy.init(args=[], domain_id=domain_id)
    node = DomainPublisher(domain_id, start_value, increment)
    try:
        # rclpy.spin으로 timer_callback이 계속 호출되도록 함
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def main():
    # domain 1: 홀수 (1부터 시작, 2씩 증가)
    p1 = multiprocessing.Process(target=publisher_process, args=(1, 1, 2))
    # domain 2: 짝수 (2부터 시작, 2씩 증가)
    p2 = multiprocessing.Process(target=publisher_process, args=(2, 2, 2))
    
    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
