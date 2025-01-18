import rclpy
from rclpy.node import Node
import tf2_ros
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class FrameTrailPublisher(Node):
    def __init__(self):
        super().__init__('frame_trail_publisher')

        # 어떤 프레임의 잔상을 그리고 싶은지 지정
        self.target_frame = 'child_frame'  # 예: 'child_frame'
        self.source_frame = 'world'        # 예: 'world'

        # TF buffer와 listener 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Marker를 퍼블리시할 Publisher 생성
        self.marker_pub = self.create_publisher(Marker, 'frame_trail_marker', 10)

        # 잔상을 저장할 리스트
        self.positions = []

        # 0.1초(=100ms)마다 TF를 조회하고 Marker를 갱신
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 현재 시각 기준 transform lookup
        # (조금 과거까지 허용하려면 timeout 인자를 조정)
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {e}")
            return

        # 변환된 좌표(translation)를 positions에 누적
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        self.positions.append((x, y, z))

        # 필요하다면, 지나치게 리스트가 길어지는 것을 방지하려고 일정 개수 이상은 버리는 로직 추가 가능
        # if len(self.positions) > 1000:
        #     self.positions.pop(0)

        # 이제 Marker 메시지 구성
        marker = Marker()
        marker.header.frame_id = self.source_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "frame_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # LINE_STRIP을 그릴 점들
        marker.points = []
        for pos in self.positions:
            p = Point()
            p.x = pos[0]
            p.y = pos[1]
            p.z = pos[2]
            marker.points.append(p)

        # 선 굵기
        marker.scale.x = 0.02  # 2cm 정도 두께

        # 색상 (rgba)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0  # 투명도

        # 수명 (0이면 영구 표시)
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FrameTrailPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
