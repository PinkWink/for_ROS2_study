import math
import rclpy
from rclpy.node import Node

import tf2_ros
from std_msgs.msg import Float32

class DistanceWorldChildPublisher(Node):
    def __init__(self):
        super().__init__('distance_world_child_publisher')
        
        # TF를 조회하기 위한 Buffer와 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 거리 정보를 발행할 Publisher
        self.dist_pub = self.create_publisher(Float32, 'distance_world_child', 10)
        
        # 주기적으로 TF를 조회해 거리 계산 (0.1초마다)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # 'world'에서 'child_frame'으로의 변환 조회
            # lookup_transform(부모, 자식, 시간)
            transform = self.tf_buffer.lookup_transform(
                'world',       # 부모 프레임(출발)
                'child_frame', # 자식 프레임(도착)
                rclpy.time.Time()
            )

            # 변환으로부터 x, y, z 추출
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # 3차원 유클리드 거리 계산
            distance = math.sqrt(x**2 + y**2 + z**2)

            # 메시지 생성 및 발행
            msg = Float32()
            msg.data = distance
            self.dist_pub.publish(msg)

        except Exception as e:
            # 아직 TF가 준비되지 않았거나, lookup 실패할 경우 예외 발생
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceWorldChildPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
