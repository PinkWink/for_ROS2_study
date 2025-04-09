#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import threading
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 모니터링용 노드: 터틀의 현재 pose, 목표 pose, 그리고 상태를 구독
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        # 터틀의 현재 pose, 목표 pose, 그리고 현재 상태 (초기에는 "idle")
        self.turtle_pose = None
        self.goal_pose = None
        self.state = "idle"
        # 가이드 선(최초 목표 수신 시점의 터틀 위치 기록)
        self.guide_line_start = None

        # 'turtle1/pose' 토픽 구독 (현재 터틀 pose)
        self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_callback, 10)
        # 'goal_pose' 토픽 구독 (목표 pose)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        # 'turtle_state' 토픽 구독 (현재 상태: 예 "rotate_to_goal", "move_to_goal", 등)
        self.create_subscription(String, 'turtle_state', self.state_callback, 10)

    def turtle_pose_callback(self, msg):
        self.turtle_pose = msg

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        # 목표 수신 시 현재 위치를 가이드 선의 시작점으로 기록
        if self.turtle_pose is not None:
            self.guide_line_start = (self.turtle_pose.x, self.turtle_pose.y)
            self.get_logger().info(
                f"Guide line set from ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}) to ({msg.x:.2f}, {msg.y:.2f})"
            )

    def state_callback(self, msg):
        self.state = msg.data
        self.get_logger().info(f"Received state: {self.state}")

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
    ax.set_title("Turtle Pose (blue arrow), Goal Pose (red dot), and Guide Lines")

    # 터틀의 현재 pose가 수신되었으면 파란색 화살표(현재 위치)와 함께 상태 텍스트 표시
    if node.turtle_pose is not None:
        x = node.turtle_pose.x
        y = node.turtle_pose.y
        theta = node.turtle_pose.theta
        arrow_length = 0.5  # 화살표 길이
        dx = arrow_length * math.cos(theta)
        dy = arrow_length * math.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        # 현재 위치를 파란 점으로 표시
        ax.plot(x, y, 'bo')
        # 상태 텍스트를 화살표 옆에 오프셋을 주어 표시
        state_text = node.state if node.state is not None else ""
        ax.text(x + 0.5, y + 0.5, state_text, color='blue', fontsize=10)

    # 목표 pose가 수신되었으면 빨간 점(원)으로 표시
    if node.goal_pose is not None:
        gx = node.goal_pose.x
        gy = node.goal_pose.y
        ax.plot(gx, gy, 'ro', markersize=8)

    # 최초 목표일 경우, 출발점과 목표점을 잇는 빨간 대시선(가이드 선) 표시
    if node.guide_line_start is not None and node.goal_pose is not None:
        start_x, start_y = node.guide_line_start
        goal_x = node.goal_pose.x
        goal_y = node.goal_pose.y
        ax.plot([start_x, goal_x], [start_y, goal_y], linestyle='--', color='red', linewidth=2)

    # rotate_to_goal 또는 rotate_to_final 상태일 때, 회전 방향 가이드 (빨간 점선의 화살표) 추가로 표시
    if (node.state in ["rotate_to_goal", "rotate_to_final"]) and node.turtle_pose is not None and node.goal_pose is not None:
        if node.state == "rotate_to_goal":
            # 터틀의 현재 위치에서 목표 위치로 향하는 각도를 계산
            angle = math.atan2(node.goal_pose.y - node.turtle_pose.y,
                               node.goal_pose.x - node.turtle_pose.x)
        elif node.state == "rotate_to_final":
            # 목표의 최종 각도 사용
            angle = node.goal_pose.theta
        guide_length = 1.0  # 회전 가이드 화살표 길이
        # 회전 가이드 화살표의 끝점 계산
        gx_line = node.turtle_pose.x + guide_length * math.cos(angle)
        gy_line = node.turtle_pose.y + guide_length * math.sin(angle)
        # annotate를 사용하여 빨간 점선 화살표 표시
        ax.annotate(
            "",
            xy=(gx_line, gy_line),
            xytext=(node.turtle_pose.x, node.turtle_pose.y),
            arrowprops=dict(arrowstyle="->", linestyle=":", color="red", lw=2)
        )

    # 터틀이 목표에 도달한 경우 (유클리드 거리 0.1 이하) 가이드 선 초기화 (사라지게 함)
    if node.turtle_pose is not None and node.goal_pose is not None:
        dist = math.sqrt((node.turtle_pose.x - node.goal_pose.x)**2 +
                         (node.turtle_pose.y - node.goal_pose.y)**2)
        if dist < 0.1:
            node.guide_line_start = None

    return ax

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitor()

    # ROS 스핀은 별도의 스레드에서 실행 (matplotlib mainloop와 충돌 방지)
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
