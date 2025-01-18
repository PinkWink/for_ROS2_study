import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class FrameTrailPublisher(Node):
    def __init__(self):
        super().__init__('frame_trail_publisher')

        # 잔상을 그리고 싶은 대상 프레임 설정
        self.target_frame = 'child_frame'  # 예) 'child_frame'
        self.source_frame = 'world'        # 예) 'world'

        # TF Buffer, Listener 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Marker를 퍼블리시할 Publisher
        self.marker_pub = self.create_publisher(Marker, 'frame_trail_marker', 10)

        # 궤적(잔상)을 저장할 리스트
        self.positions = []

        # 몇 개의 점만 유지할지(슬라이딩 윈도우 크기)
        self.max_points = 20

        # 0.1초 간격으로 TF를 확인하고 Marker 갱신
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 현재 시각 기준 world->child_frame 변환 조회
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform {self.source_frame} -> {self.target_frame}: {e}")
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

        # 좌표를 리스트에 추가
        self.positions.append((x, y, z))

        # 슬라이딩 윈도우: 초과분은 앞에서부터 제거
        if len(self.positions) > self.max_points:
            self.positions.pop(0)

        # Marker 생성
        marker = Marker()
        marker.header.frame_id = self.source_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "frame_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Marker에 들어갈 점들
        marker.points = []
        for pos in self.positions:
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            marker.points.append(point)

        # 선(라인) 굵기
        marker.scale.x = 0.1  

        # 색상(빨간색)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0  # 불투명

        # lifetime이 0이면 영구 표시
        marker.lifetime.sec = 0

        # 퍼블리시
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FrameTrailPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
