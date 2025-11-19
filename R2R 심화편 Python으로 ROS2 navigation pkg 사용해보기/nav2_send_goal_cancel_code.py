#!/usr/bin/env python3
import math
import time
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quaternion_from_yaw(yaw_rad: float):
    """yaw(rad) -> (z, w) 쿼터니언 변환"""
    z = math.sin(yaw_rad / 2.0)
    w = math.cos(yaw_rad / 2.0)
    return z, w


def main():

    # -----------------------------
    # CLI 인자 파싱
    # python3 nav2_send_goal_basic.py x y yaw_deg
    # -----------------------------
    if len(sys.argv) != 5:
        print("Usage: python3 nav2_send_goal_cancel_code.py <x> <y> <yaw_deg> <duration_time>")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
        CANCEL_AFTER_SEC = float(sys.argv[4])
    except ValueError:
        print("Error: x, y, yaw_deg, time must be numbers")
        return


    rclpy.init(args=None)

    # Nav2 네비게이터 생성
    navigator = BasicNavigator()

    print("[Nav2] 활성화 대기 중...")
    navigator.waitUntilNav2Active()
    print("[Nav2] Active 상태입니다.")

    # goal pose 생성
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y

    yaw_rad = math.radians(yaw_deg)
    goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = quaternion_from_yaw(
        yaw_rad
    )

    print(f"[Goal] ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg) 로 이동 시작")
    navigator.goToPose(goal_pose)

    start_time = time.time()

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            elapsed = time.time() - start_time
            print(
                f"[Feedback] 남은 거리: {feedback.distance_remaining:.2f} m, "
                f"경과 시간: {feedback.navigation_time.sec} s "
                f"(실시간 {elapsed:.1f} s)"
            )

            if elapsed > CANCEL_AFTER_SEC:
                print(f"[Action] {CANCEL_AFTER_SEC}초 초과 → Goal 취소 요청")
                navigator.cancelTask()
                break

        time.sleep(0.5)

    # 최종 결과
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[Result] 성공적으로 도착했습니다.")
    elif result == TaskResult.CANCELED:
        print("[Result] Goal이 취소되었습니다.")
    elif result == TaskResult.FAILED:
        print("[Result] Goal 수행에 실패했습니다.")
    else:
        print("[Result] 알 수 없는 상태:", result)

    result = navigator.getResult()
    print("[Result] TaskResult =", result)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
