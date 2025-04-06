#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import threading
import math
import sys

# PyQt5와 matplotlib 임포트
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ROS 노드: 터틀의 현재 pose와 목표 pose를 구독 및 goal_pose 발행 기능 추가
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        self.turtle_pose = None      # 현재 터틀의 pose
        self.goal_pose = None        # 목표 pose
        self.guide_line_start = None # 가이드 선 시작점

        # 'turtle1/pose' 토픽 구독 (현재 터틀 pose)
        self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_callback, 10)
        # 'goal_pose' 토픽 구독 (목표 pose)
        self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        
        # 마우스 클릭으로 goal_pose를 발행하기 위한 퍼블리셔 추가
        self.goal_pub = self.create_publisher(Pose, 'goal_pose', 10)

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

# PyQt 메인 윈도우 클래스: 맵과 두 개의 그래프 영역 포함
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Turtle Monitor with Coordinate Graphs")

        # 시간 및 좌표 기록을 위한 변수 초기화
        self.time_counter = 0.0
        self.time_history = []
        self.x_history = []
        self.goal_x_history = []
        self.y_history = []
        self.goal_y_history = []

        # 메인 레이아웃: 왼쪽에 맵, 오른쪽에 두 개의 그래프 (수직 배치)
        main_widget = QWidget()
        main_layout = QHBoxLayout()

        # 왼쪽: 터틀 맵 (matplotlib 캔버스)
        self.figure_map = Figure()
        self.canvas = FigureCanvas(self.figure_map)
        self.ax = self.figure_map.add_subplot(111)
        main_layout.addWidget(self.canvas, stretch=1)  

        # 맵 캔버스에 클릭 이벤트 연결: 마우스로 클릭하면 좌표를 goal_pose로 발행
        self.canvas.mpl_connect('button_press_event', self.on_map_click)

        # 오른쪽: x, y 좌표 그래프를 담을 위젯 (수직 배치)
        graph_widget = QWidget()
        graph_layout = QVBoxLayout()

        # x 좌표 그래프 캔버스
        self.figure_x = Figure(figsize=(4, 2))
        self.figure_x.subplots_adjust(right=0.75)
        self.canvas_x = FigureCanvas(self.figure_x)
        self.ax_x = self.figure_x.add_subplot(111)
        graph_layout.addWidget(self.canvas_x)

        # y 좌표 그래프 캔버스
        self.figure_y = Figure(figsize=(4, 2))
        self.figure_y.subplots_adjust(right=0.75)
        self.canvas_y = FigureCanvas(self.figure_y)
        self.ax_y = self.figure_y.add_subplot(111)
        graph_layout.addWidget(self.canvas_y)

        graph_widget.setLayout(graph_layout)
        main_layout.addWidget(graph_widget, stretch=2)  # 그래프 영역을 더 크게

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # QTimer를 이용해 100ms 간격으로 모든 플롯 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

    def on_map_click(self, event):
        # 클릭 이벤트가 맵 영역에서 발생했는지 확인
        if event.inaxes == self.ax:
            x_click = event.xdata
            y_click = event.ydata

            # 클릭한 좌표를 goal_pose 메시지로 생성 (theta는 0으로 설정)
            goal_msg = Pose()
            goal_msg.x = x_click
            goal_msg.y = y_click
            goal_msg.theta = 0.0

            # 퍼블리셔를 통해 goal_pose 토픽으로 발행
            self.node.goal_pub.publish(goal_msg)
            self.node.get_logger().info(f"Published new goal: ({x_click:.2f}, {y_click:.2f})")

    def update_all(self):
        # 타임스탬프 업데이트 (0.1초 간격 가정)
        self.time_counter += 0.1
        self.time_history.append(self.time_counter)

        # 터틀의 현재 좌표 업데이트
        if self.node.turtle_pose is not None:
            current_x = self.node.turtle_pose.x
            current_y = self.node.turtle_pose.y
        else:
            current_x = self.x_history[-1] if self.x_history else 0
            current_y = self.y_history[-1] if self.y_history else 0
        self.x_history.append(current_x)
        self.y_history.append(current_y)

        # 목표 좌표 업데이트 (없으면 이전 값 유지)
        if self.node.goal_pose is not None:
            goal_x = self.node.goal_pose.x
            goal_y = self.node.goal_pose.y
        else:
            goal_x = self.goal_x_history[-1] if self.goal_x_history else 0
            goal_y = self.goal_y_history[-1] if self.goal_y_history else 0
        self.goal_x_history.append(goal_x)
        self.goal_y_history.append(goal_y)

        # 맵 업데이트
        self.ax.clear()
        self.ax.set_xlim(0, 11)
        self.ax.set_ylim(0, 11)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title("Turtle MAP")

        # 현재 터틀 pose (파란 화살표 및 점)
        if self.node.turtle_pose is not None:
            x = self.node.turtle_pose.x
            y = self.node.turtle_pose.y
            theta = self.node.turtle_pose.theta
            arrow_length = 0.5
            dx = arrow_length * math.cos(theta)
            dy = arrow_length * math.sin(theta)
            self.ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
            self.ax.plot(x, y, 'bo')
        # 목표 pose (빨간 점)
        if self.node.goal_pose is not None:
            gx = self.node.goal_pose.x
            gy = self.node.goal_pose.y
            self.ax.plot(gx, gy, 'ro', markersize=8)
        # 가이드 선: 목표 수신 시 기록된 시작점에서 목표까지 빨간 점선
        if self.node.guide_line_start is not None and self.node.goal_pose is not None:
            start_x, start_y = self.node.guide_line_start
            goal_x = self.node.goal_pose.x
            goal_y = self.node.goal_pose.y
            self.ax.plot([start_x, goal_x], [start_y, goal_y], 'r--')
        # 터틀이 목표에 도달하면 가이드 선 초기화 (유클리드 거리 0.1 이하)
        if self.node.turtle_pose is not None and self.node.goal_pose is not None:
            dist = math.sqrt((self.node.turtle_pose.x - self.node.goal_pose.x) ** 2 +
                             (self.node.turtle_pose.y - self.node.goal_pose.y) ** 2)
            if dist < 0.1:
                self.node.guide_line_start = None

        self.canvas.draw()

        # x 좌표 그래프 업데이트 (최근 10초간의 데이터만 표시)
        self.ax_x.clear()
        self.ax_x.plot(self.time_history, self.x_history, label="Current X", color='blue')
        self.ax_x.plot(self.time_history, self.goal_x_history, label="Goal X", color='red', 
                                                                                linestyle='--')
        self.ax_x.set_title("X Coordinate")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("X Value")
        self.ax_x.grid(True)
        if self.time_counter < 10:
            self.ax_x.set_xlim(0, 10)
        else:
            self.ax_x.set_xlim(self.time_counter - 10, self.time_counter)
        self.ax_x.legend(loc='upper left', bbox_to_anchor=(1, 1))
        self.canvas_x.draw()

        # y 좌표 그래프 업데이트 (최근 10초간의 데이터만 표시)
        self.ax_y.clear()
        self.ax_y.plot(self.time_history, self.y_history, label="Current Y", color='blue')
        self.ax_y.plot(self.time_history, self.goal_y_history, label="Goal Y", color='red', 
                                                                                linestyle='--')
        self.ax_y.set_title("Y Coordinate")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Y Value")
        self.ax_y.grid(True)
        if self.time_counter < 10:
            self.ax_y.set_xlim(0, 10)
        else:
            self.ax_y.set_xlim(self.time_counter - 10, self.time_counter)
        self.ax_y.legend(loc='upper left', bbox_to_anchor=(1, 1))
        self.canvas_y.draw()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitor()

    # ROS 스핀을 별도의 스레드에서 실행
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # PyQt 애플리케이션 실행
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
