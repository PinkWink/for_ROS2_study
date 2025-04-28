#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import threading
import math
import sys

# PyQt5와 matplotlib 임포트
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import FancyBboxPatch  # 라운드 테두리 처리를 위해

# ROS 노드: 터틀의 현재 pose, goal_pose, state 구독 및 goal_pose 발행
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        self.turtle_pose = None      # 현재 터틀의 pose
        self.goal_pose = None        # 현재 goal_pose
        self.current_state = None    # 현재 상태 (문자열)
        self.guide_line_start = None # 가이드 선의 시작점 (터틀의 pose)

        # 구독자 설정
        self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_callback, 10)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(String, 'state', self.state_callback, 10)

        # 퍼블리셔: 마우스 드래그로 새 goal_pose 발행
        self.goal_pub = self.create_publisher(Pose, 'goal_pose', 10)

    def turtle_pose_callback(self, msg):
        self.turtle_pose = msg

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        if self.turtle_pose is not None:
            self.guide_line_start = (self.turtle_pose.x, self.turtle_pose.y)
            self.get_logger().info(
                f"Guide line set from ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}) to ({msg.x:.2f}, {msg.y:.2f})"
            )

    def state_callback(self, msg):
        self.current_state = msg.data


def ros_spin(node):
    rclpy.spin(node)


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Turtle Monitor with Goal-Drag and State Display")

        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # [좌측] 터틀 맵 영역
        self.figure_map = Figure()
        self.canvas_map = FigureCanvas(self.figure_map)
        self.ax_map = self.figure_map.add_subplot(111)
        main_layout.addWidget(self.canvas_map, stretch=3)

        # [우측] 상태 표시 영역 (세로 크기 3x6)
        self.figure_state = Figure(figsize=(3, 6))
        self.canvas_state = FigureCanvas(self.figure_state)
        self.ax_state = self.figure_state.add_subplot(111)
        main_layout.addWidget(self.canvas_state, stretch=1)

        self.drag_start = None
        self.drag_current = None

        self.canvas_map.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas_map.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas_map.mpl_connect('button_release_event', self.on_mouse_release)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        # 컨트롤러에서 발행하는 상태와 일치 (상태 목록)
        self.state_list = ["RotateToGoal", "MoveToGoal", "RotateToFinal", "GoalReached"]

    def on_mouse_press(self, event):
        if event.button == 1 and event.inaxes == self.ax_map:
            self.drag_start = (event.xdata, event.ydata)
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_move(self, event):
        if self.drag_start is not None and event.inaxes == self.ax_map:
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_release(self, event):
        if event.button == 1 and self.drag_start is not None and event.inaxes == self.ax_map:
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
            self.drag_start = None
            self.drag_current = None

    def update_all(self):
        self.update_map()
        self.update_state_display()

    def update_map(self):
        self.ax_map.clear()
        self.ax_map.set_xlim(0, 11)
        self.ax_map.set_ylim(0, 11)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True)
        self.ax_map.set_title("Turtle Map")

        if self.node.turtle_pose is not None:
            x = self.node.turtle_pose.x
            y = self.node.turtle_pose.y
            theta = self.node.turtle_pose.theta
            arrow_len = 0.5
            dx = arrow_len * math.cos(theta)
            dy = arrow_len * math.sin(theta)
            self.ax_map.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
            self.ax_map.plot(x, y, 'bo')

        if self.node.goal_pose is not None:
            gx = self.node.goal_pose.x
            gy = self.node.goal_pose.y
            gtheta = self.node.goal_pose.theta
            self.ax_map.plot(gx, gy, 'ro', markersize=8)
            arrow_len = 0.5
            dx = arrow_len * math.cos(gtheta)
            dy = arrow_len * math.sin(gtheta)
            self.ax_map.arrow(gx, gy, dx, dy, head_width=0.3, head_length=0.3, fc='red', ec='red')

        if self.drag_start is not None and self.drag_current is not None:
            sx, sy = self.drag_start
            cx, cy = self.drag_current
            self.ax_map.arrow(  sx, sy, cx - sx, cy - sy, head_width=0.3, head_length=0.3,
                                fc='green', ec='green', linestyle='--')
            
        if self.node.guide_line_start is not None and self.node.goal_pose is not None:
            start_x, start_y = self.node.guide_line_start
            goal_x = self.node.goal_pose.x
            goal_y = self.node.goal_pose.y
            self.ax_map.plot([start_x, goal_x], [start_y, goal_y], 'r--')
            if self.node.turtle_pose is not None:
                dist = math.sqrt(   (self.node.turtle_pose.x - goal_x)**2 +
                                    (self.node.turtle_pose.y - goal_y)**2)
                if dist < 0.1:
                    self.node.guide_line_start = None

        self.canvas_map.draw()

    def update_state_display(self):
        self.ax_state.clear()
        # 오른쪽 영역: x축 0 ~ 1.5, y축 0 ~ 1
        self.ax_state.set_xlim(0, 1.5)
        self.ax_state.set_ylim(0, 1)
        self.ax_state.axis('off')

        block_width = 0.8 * 1.2      # 0.96
        block_height = 0.06 * 1.1    # 약 0.066
        spacing = 0.12             # 블록 간 간격 (화살표 길이)
        
        # 오른쪽 영역의 폭은 1.5이므로, 가로 중앙에 배치: start_x = (1.5 - block_width) / 2
        start_x = (1.5 - block_width) / 2  # 약 0.27

        # 블록 전체 그룹을 수직 중앙 정렬하기 위해 총 높이를 계산
        n = len(self.state_list)
        total_height = n * block_height + (n - 1) * spacing
        group_bottom = 0.5 - total_height / 2
        group_top = group_bottom + total_height

        boundaries = []
        active_color = '#A2D2FF'    # 파스텔 블루
        inactive_color = '#E9ECEF'  # 연한 파스텔 그레이

        # 블록들을 그룹 중앙 기준으로 위에서부터 차례로 배치
        for i, state in enumerate(self.state_list):
            # i=0: 최상단 블록, 각 블록의 바닥 y 좌표
            y = group_top - (i + 1) * block_height - i * spacing
            face_color = active_color if self.node.current_state == state else inactive_color
            rect = FancyBboxPatch(( start_x, y), block_width, block_height,
                                    boxstyle="round,pad=0.02",
                                    fc=face_color, ec="black", lw=1.5)
            self.ax_state.add_patch(rect)
            self.ax_state.text(  start_x + block_width/2, y + block_height/2,
                                state, horizontalalignment='center',
                                verticalalignment='center',
                                color='black', fontsize=10)
            top_center = (start_x + block_width/2, y + block_height)
            bottom_center = (start_x + block_width/2, y)
            boundaries.append((top_center, bottom_center))
        
        for i in range(len(boundaries) - 1):
            start_point = boundaries[i][1]
            end_point = boundaries[i+1][0]
            self.ax_state.annotate( "",
                                    xy=end_point, xycoords='data',
                                    xytext=start_point, textcoords='data',
                                    arrowprops=dict(arrowstyle="->", color='black'))
        self.canvas_state.draw()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitor()
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
