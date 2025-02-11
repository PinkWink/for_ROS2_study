import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class FramePathPublisher(Node):
    def __init__(self):
        super().__init__('frame_path_publisher')
        self.target_frame = 'child_frame'
        self.source_frame = 'world'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, 'frame_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.source_frame

        # 최대 Pose 갯수 설정 (필요에 따라 값 조정)
        self.max_poses = 50

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # transform lookup
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {e}")
            return

        # 새로운 PoseStamped 생성 후 Path에 추가
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.source_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation

        self.path_msg.poses.append(pose)

        # Pose 리스트 길이가 최대 갯수를 초과하면 가장 오래된 Pose 삭제
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        self.path_msg.header.stamp = pose.header.stamp

        # Path Publish
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FramePathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
