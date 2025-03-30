#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Float64
from turtlesim.msg import Pose

class WebPublisherNode(Node):
    def __init__(self):
        super().__init__('web_publisher_node')
        # 내부 상태 변수 초기화
        self.state = None
        self.angle_error = None
        self.distance_error = None
        self.turtle_pose = None
        self.goal_pose = None
        
        # 기존 노드에서 발행하는 토픽들을 구독합니다.
        self.create_subscription(String, 'state', self.state_callback, 10)
        self.create_subscription(Float64, 'angle_error', self.angle_error_callback, 10)
        self.create_subscription(Float64, 'distance_error', self.distance_error_callback, 10)
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        
        # 웹 전송용 토픽 발행 (rosbridge를 통해 웹소켓 연결)
        self.publisher = self.create_publisher(String, 'web/aggregated_state', 10)
        
        # 10Hz로 최신 상태를 발행합니다.
        self.timer = self.create_timer(0.1, self.publish_data)

    def state_callback(self, msg: String):
        self.state = msg.data

    def angle_error_callback(self, msg: Float64):
        self.angle_error = msg.data

    def distance_error_callback(self, msg: Float64):
        self.distance_error = msg.data

    def pose_callback(self, msg: Pose):
        self.turtle_pose = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta,
            'linear_velocity': msg.linear_velocity,
            'angular_velocity': msg.angular_velocity
        }

    def goal_pose_callback(self, msg: Pose):
        self.goal_pose = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta,
            'linear_velocity': msg.linear_velocity,
            'angular_velocity': msg.angular_velocity
        }

    def publish_data(self):
        data = {
            'state': self.state,
            'angle_error': self.angle_error,
            'distance_error': self.distance_error,
            'turtle_pose': self.turtle_pose,
            'goal_pose': self.goal_pose
        }
        json_data = json.dumps(data)
        msg = String()
        msg.data = json_data
        self.publisher.publish(msg)
        self.get_logger().debug("Published aggregated state to web")

def main(args=None):
    rclpy.init(args=args)
    node = WebPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("WebPublisherNode interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
