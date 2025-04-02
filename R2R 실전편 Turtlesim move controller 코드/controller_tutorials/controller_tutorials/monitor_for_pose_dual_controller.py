#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import threading
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 모니터링용 노드: 터틀의 현재 pose와 목표 pose를 구독
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        # 터틀 현재 pose와 목표 pose 저장 (초기에는 None)
        self.turtle_pose = None
        self.goal_pose = None
        # 가이드 선을 위해 목표 수신 시점의 터틀 위치 (튜플: (x, y))
        self.guide_line_start = None

        # 'turtle1/pose' 토픽 구독 (현재 터틀 pose)
        self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_callback, 10)
        # 'goal_pose' 토픽 구독 (목표 pose)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)

    def turtle_pose_callback(self, msg):
        self.turtle_pose = msg

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        # 목표 토픽 수신 시점에 터틀의 위치를 기록하여 가이드 선의 시작점으로 저장
        if self.turtle_pose is not None:
            self.guide_line_start = (self.turtle_pose.x, self.turtle_pose.y)
            self.get_logger().info(
                f"Guide line set from ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}) to ({msg.x:.2f}, {msg.y:.2f})"
            )

# ROS 스핀을 별도 스레드에서 실행하기 위한 함수
def ros_spin(node):
    rclpy.spin(node)

# 애니메이션 업데이트 함수
def update(frame, node, ax):
    # 그래프 초기화
    ax.clear()
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 11)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title("Turtle Pose (blue arrow), Goal Pose (red dot) and Guide Line (red dashed)")

    # 터틀의 현재 pose가 수신되었으면 파란색 화살표로 표시
    if node.turtle_pose is not None:
        x = node.turtle_pose.x
        y = node.turtle_pose.y
        theta = node.turtle_pose.theta
        arrow_length = 0.5  # 화살표 길이
        dx = arrow_length * math.cos(theta)
        dy = arrow_length * math.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        # 현재 위치를 파란 점으로도 표시 (옵션)
        ax.plot(x, y, 'bo')

    # 목표 pose가 수신되었으면 빨간 점으로 표시
    if node.goal_pose is not None:
        gx = node.goal_pose.x
        gy = node.goal_pose.y
        ax.plot(gx, gy, 'ro', markersize=8)

    # 가이드 선이 설정되어 있다면, 기록된 시작점과 목표점을 잇는 빨간 점선으로 표시
    if node.guide_line_start is not None and node.goal_pose is not None:
        start_x, start_y = node.guide_line_start
        goal_x = node.goal_pose.x
        goal_y = node.goal_pose.y
        ax.plot([start_x, goal_x], [start_y, goal_y], 'r--')

    # 만약 터틀이 목표에 도달했다면 (예, 0.1 이하의 유클리드 거리)
    if node.turtle_pose is not None and node.goal_pose is not None:
        dist = math.sqrt((node.turtle_pose.x - node.goal_pose.x) ** 2 +
                         (node.turtle_pose.y - node.goal_pose.y) ** 2)
        if dist < 0.1:
            # 목표에 도달하면 가이드 선 초기화 (사라지게 함)
            node.guide_line_start = None

    return ax

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitor()

    # ROS 스핀은 별도의 스레드에서 실행
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # matplotlib figure 생성 및 애니메이션 설정
    fig, ax = plt.subplots()
    ani = animation.FuncAnimation(fig, update, fargs=(node, ax), interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
