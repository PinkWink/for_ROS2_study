#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import threading
import math
import sys

# PyQt5와 matplotlib 임포트
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import Rectangle

# ROS 노드: 터틀의 현재 pose, goal_pose, state를 구독하며 goal_pose를 발행
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        self.turtle_pose = None      # 현재 터틀의 pose
        self.goal_pose = None        # 현재 goal_pose
        self.current_state = None    # 현재 상태 (문자열)
        self.guide_line_start = None # 가이드 선의 시작점 (터틀의 pose)

        # 구독자: 터틀의 현재 pose, goal_pose, 그리고 state 정보를 받음
        self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_callback, 10)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(String, 'state', self.state_callback, 10)

        # 퍼블리셔: 마우스 드래그로 새 goal_pose를 발행
        self.goal_pub = self.create_publisher(Pose, 'goal_pose', 10)

    def turtle_pose_callback(self, msg):
        self.turtle_pose = msg

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        # 목표 토픽 수신 시점에 터틀의 위치를 가이드 선 시작점으로 기록
        if self.turtle_pose is not None:
            self.guide_line_start = (self.turtle_pose.x, self.turtle_pose.y)
            self.get_logger().info(
                f"Guide line set from ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}) to ({msg.x:.2f}, {msg.y:.2f})"
            )

    def state_callback(self, msg):
        self.current_state = msg.data

# ROS 스핀을 별도의 스레드에서 실행하기 위한 함수
def ros_spin(node):
    rclpy.spin(node)

# PyQt 메인 윈도우: 좌측은 터틀 맵, 우측은 상태(state) 블록을 표시
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Turtle Monitor with Goal-Drag, State Display & Guide Line")

        # 전체 레이아웃: 좌측(터틀 맵)과 우측(상태 블록)
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # [좌측] 터틀 맵 영역 (Matplotlib 캔버스)
        self.figure_map = Figure()
        self.canvas_map = FigureCanvas(self.figure_map)
        self.ax_map = self.figure_map.add_subplot(111)
        main_layout.addWidget(self.canvas_map, stretch=3)

        # [우측] 상태 표시 영역 (Matplotlib 캔버스)
        self.figure_state = Figure(figsize=(3, 4))
        self.canvas_state = FigureCanvas(self.figure_state)
        self.ax_state = self.figure_state.add_subplot(111)
        main_layout.addWidget(self.canvas_state, stretch=1)

        # 마우스 드래그 관련 변수
        self.drag_start = None   # 마우스 왼쪽 버튼 클릭 시 기록 (goal_pose의 위치)
        self.drag_current = None # 버튼을 누른 상태에서의 현재 마우스 위치 (방향 결정)

        # Matplotlib의 마우스 이벤트 연결
        self.canvas_map.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas_map.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas_map.mpl_connect('button_release_event', self.on_mouse_release)

        # 100ms 간격으로 플롯 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        # 상태 순서: state machine에 따른 순서 (예시)
        self.state_list = ["RotateToGoal", "MoveToGoal", "RotateToFinal", "GoalReached"]

    def on_mouse_press(self, event):
        # 왼쪽 버튼 클릭 시, 해당 위치를 goal_pose의 x, y 좌표로 기록
        if event.button == 1 and event.inaxes == self.ax_map:
            self.drag_start = (event.xdata, event.ydata)
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_move(self, event):
        # 드래그 중이면 현재 마우스 위치 업데이트
        if self.drag_start is not None and event.inaxes == self.ax_map:
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_release(self, event):
        # 왼쪽 버튼 해제 시, goal_pose 발행 (목표 위치는 클릭한 시작점)
        if event.button == 1 and self.drag_start is not None and event.inaxes == self.ax_map:
            # 방향은 드래그 시작점과 현재 마우스 위치(드래그 중 마지막 위치)로 계산
            if self.drag_current is not None:
                dx = self.drag_current[0] - self.drag_start[0]
                dy = self.drag_current[1] - self.drag_start[1]
                theta = math.atan2(dy, dx)
            else:
                theta = 0.0

            goal_msg = Pose()
            goal_msg.x = self.drag_start[0]
            goal_msg.y = self.drag_start[1]
            goal_msg.theta = theta
            self.node.goal_pub.publish(goal_msg)
            self.node.get_logger().info(
                f"Published new goal: x={goal_msg.x:.2f}, y={goal_msg.y:.2f}, theta={goal_msg.theta:.2f}"
            )
            # 드래그 변수 초기화
            self.drag_start = None
            self.drag_current = None

    def update_all(self):
        self.update_map()
        self.update_state_display()

    def update_map(self):
        # 터틀 맵 업데이트
        self.ax_map.clear()
        self.ax_map.set_xlim(0, 11)
        self.ax_map.set_ylim(0, 11)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True)
        self.ax_map.set_title("Turtle Map")

        # 터틀 pose (파란 화살표 및 점)
        if self.node.turtle_pose is not None:
            x = self.node.turtle_pose.x
            y = self.node.turtle_pose.y
            theta = self.node.turtle_pose.theta
            arrow_len = 0.5
            dx = arrow_len * math.cos(theta)
            dy = arrow_len * math.sin(theta)
            self.ax_map.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
            self.ax_map.plot(x, y, 'bo')
        # goal_pose (빨간 점과 화살표)
        if self.node.goal_pose is not None:
            gx = self.node.goal_pose.x
            gy = self.node.goal_pose.y
            gtheta = self.node.goal_pose.theta
            self.ax_map.plot(gx, gy, 'ro', markersize=8)
            arrow_len = 0.5
            dx = arrow_len * math.cos(gtheta)
            dy = arrow_len * math.sin(gtheta)
            self.ax_map.arrow(gx, gy, dx, dy, head_width=0.3, head_length=0.3, fc='red', ec='red')
        # 드래그 중이면, 시작점부터 현재 마우스 위치까지 화살표 표시
        if self.drag_start is not None and self.drag_current is not None:
            sx, sy = self.drag_start
            cx, cy = self.drag_current
            self.ax_map.arrow(sx, sy, cx - sx, cy - sy, head_width=0.3, head_length=0.3,
                              fc='green', ec='green', linestyle='--')
        # 가이드 선: 가이드 시작점에서 goal_pose까지 빨간 점선으로 표시
        if self.node.guide_line_start is not None and self.node.goal_pose is not None:
            start_x, start_y = self.node.guide_line_start
            goal_x = self.node.goal_pose.x
            goal_y = self.node.goal_pose.y
            self.ax_map.plot([start_x, goal_x], [start_y, goal_y], 'r--')
            # 터틀이 목표에 도달하면 가이드 선 초기화 (유클리드 거리 0.1 이하)
            if self.node.turtle_pose is not None:
                dist = math.sqrt((self.node.turtle_pose.x - goal_x)**2 + (self.node.turtle_pose.y - goal_y)**2)
                if dist < 0.1:
                    self.node.guide_line_start = None

        self.canvas_map.draw()

    def update_state_display(self):
        # 상태 블록 업데이트 (블록 경계에서 화살표가 출발·도착하도록)
        self.ax_state.clear()
        self.ax_state.set_xlim(0, 1)
        self.ax_state.set_ylim(0, 1)
        self.ax_state.axis('off')

        block_width = 0.8
        block_height = 0.15
        spacing = 0.05
        start_x = 0.1
        start_y = 1 - block_height - 0.1

        boundaries = []  # 각 블록의 (상단 중앙, 하단 중앙) 좌표 저장
        for i, state in enumerate(self.state_list):
            y = start_y - i * (block_height + spacing)
            face_color = 'blue' if self.node.current_state == state else 'gray'
            rect = Rectangle((start_x, y), block_width, block_height, facecolor=face_color, edgecolor='black')
            self.ax_state.add_patch(rect)
            self.ax_state.text(start_x + block_width/2, y + block_height/2, state,
                               horizontalalignment='center', verticalalignment='center',
                               color='white', fontsize=10)
            top_center = (start_x + block_width/2, y + block_height)  # 블록 상단 중앙
            bottom_center = (start_x + block_width/2, y)               # 블록 하단 중앙
            boundaries.append((top_center, bottom_center))
        # 블록들을 연결하는 화살표: 위 블록의 하단에서 바로 아래 블록의 상단으로
        for i in range(len(boundaries) - 1):
            start_point = boundaries[i][1]
            end_point = boundaries[i+1][0]
            self.ax_state.annotate("",
                                   xy=end_point, xycoords='data',
                                   xytext=start_point, textcoords='data',
                                   arrowprops=dict(arrowstyle="->", color='black'))
        self.canvas_state.draw()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitor()

    # ROS 스핀은 별도 스레드에서 실행
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
