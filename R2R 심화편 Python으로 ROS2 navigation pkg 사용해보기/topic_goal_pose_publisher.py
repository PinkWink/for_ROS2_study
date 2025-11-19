#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quaternion_from_yaw(yaw_rad: float):
    z = math.sin(yaw_rad / 2.0)
    w = math.cos(yaw_rad / 2.0)
    return z, w


class GoalPosePublisher(Node):
    """
    /goal_pose 토픽으로 PoseStamped를 Publish하는 예제
    """

    def __init__(self):
        super().__init__("pinky_goal_pose_publisher")
        self.publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.get_logger().info("goal_pose 퍼블리셔 준비 완료")

    def make_msg(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.z, msg.pose.orientation.w = quaternion_from_yaw(yaw_rad)

        return msg

    def send_goal_pose(self, x: float, y: float, yaw_deg: float):
        msg = self.make_msg(x, y, yaw_deg)
        self.publisher.publish(msg)
        self.get_logger().info(
            f"goal_pose 전송: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f} deg"
        )


def main():
    # -----------------------------
    # CLI 인자 파싱
    # python3 topic_goal_pose_publisher.py x y yaw_deg
    # -----------------------------
    if len(sys.argv) != 4:
        print("Usage: python3 topic_goal_pose_publisher.py <x> <y> <yaw_deg>")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
    except ValueError:
        print("Error: x, y, yaw_deg must be numbers")
        return

    rclpy.init(args=None)
    node = GoalPosePublisher()

    # ---- 1) subscriber 붙을 때까지 기다리기 ----
    wait_sec = 0.0
    timeout = 2.0  # 최대 2초 기다림

    while node.publisher.get_subscription_count() == 0 and wait_sec < timeout:
        node.get_logger().info(
            f"subscriber 대기 중... (현재 {node.publisher.get_subscription_count()}명, {wait_sec:.1f}s 경과)"
        )
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_sec += 0.1

    node.get_logger().info(
        f"최종 subscriber 수: {node.publisher.get_subscription_count()}"
    )

    # ---- 2) goal_pose 여러 번 발행 ----
    publish_count = 10
    for i in range(publish_count):
        node.send_goal_pose(x, y, yaw_deg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.get_logger().info("goal_pose 전송 완료, 노드 종료")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
