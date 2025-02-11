import math

import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

class FramePathPublisher(Node):
    def __init__(self):
        super().__init__('frame_path_publisher')
        self.target_frame = 'child_frame'
        self.source_frame = 'world'

        # TF2 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Path publisher (기존)
        self.path_pub = self.create_publisher(Path, 'frame_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.source_frame

        # 최대 Pose 갯수 설정
        self.max_poses = 50

        # 거리를 발행하기 위한 publisher
        self.distance_pub = self.create_publisher(Float64, 'child_frame_distance', 10)
        self.marker_pub = self.create_publisher(Marker, 'distance_marker', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # TF 변환 lookup
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {e}")
            return

        # 기존: Path에 PoseStamped 추가
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.source_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)
        self.path_msg.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path_msg)

        # world 기준 child_frame의 거리 계산 (유클리드 거리)
        dx = trans.transform.translation.x
        dy = trans.transform.translation.y
        dz = trans.transform.translation.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # 1) Float64 메시지로 거리 발행
        dist_msg = Float64()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)

        # 2) Marker 메시지로 텍스트 형태의 거리 발행 (rviz2에서 시각화)
        marker = Marker()
        marker.header.frame_id = self.source_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "distance"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Marker를 child_frame 바로 위에 표시하도록 위치 설정 (약간 z축으로 오프셋)
        marker.pose.position.x = dx
        marker.pose.position.y = dy
        marker.pose.position.z = dz + 0.5

        # 텍스트 크기와 색상 설정
        marker.scale.z = 0.5  # 텍스트 높이
        marker.color.a = 1.0  # 투명도 (1.0: 불투명)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.text = f"Distance: {distance:.2f} m"
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FramePathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
