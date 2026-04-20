#!/usr/bin/env python3
import rclpy
import sys  # 터미널에서 입력한 숫자를 받아오기 위한 모듈
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import os

def set_crab_mode(navigator, enable):
    if enable:
        navigator.info("Switching to CRAB mode (Omni).")
        os.system("ros2 param set /controller_server FollowPath.wz_max 0.001 > /dev/null 2>&1")
        os.system("ros2 param set /controller_server FollowPath.vy_max 0.5 > /dev/null 2>&1")
        os.system("ros2 param set /controller_server FollowPath.vy_min -0.5 > /dev/null 2>&1")
    else:
        navigator.info("Switching to NORMAL mode (Diff-drive).")
        os.system("ros2 param set /controller_server FollowPath.wz_max 1.9 > /dev/null 2>&1")
        os.system("ros2 param set /controller_server FollowPath.vy_max 0.0 > /dev/null 2>&1")
        os.system("ros2 param set /controller_server FollowPath.vy_min 0.0 > /dev/null 2>&1")

def main():
    # 터미널 입력값(X, Y)이 없으면 안내 메시지 띄우고 종료
    if len(sys.argv) < 3:
        print("[ERROR] Usage: ros2 run nar_amr_navigation test_crab_walk.py <Target_X> <Target_Y>")
        print("[Example] ros2 run nar_amr_navigation test_crab_walk.py 3.0 1.5")
        sys.exit(1)

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])

    rclpy.init()
    navigator = BasicNavigator()

    navigator.info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    navigator.info(f"Nav2 is active. Starting test for X={target_x}, Y={target_y}")

    set_crab_mode(navigator, False)
    time.sleep(1.0)

    # 1단계 목표: 터미널에서 받은 X 좌표로 직진, Y는 0 유지
    goal_1 = PoseStamped()
    goal_1.header.frame_id = 'map'
    goal_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_1.pose.position.x = target_x
    goal_1.pose.position.y = 0.0
    goal_1.pose.orientation.w = 1.0

    # 2단계 목표: 직진한 상태에서 터미널에서 받은 Y 좌표로 게걸음
    goal_2 = PoseStamped()
    goal_2.header.frame_id = 'map'
    goal_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_2.pose.position.x = target_x
    goal_2.pose.position.y = target_y
    goal_2.pose.orientation.w = 1.0

    # --- 1단계 주행 ---
    navigator.info("Stage 1: Moving forward.")
    navigator.goToPose(goal_1)
    
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.info(f"Stage 1 - Distance remaining: {feedback.distance_remaining:.2f} m", throttle_duration_sec=1.0)
        time.sleep(0.1)

    if navigator.getResult() == TaskResult.SUCCEEDED:
        navigator.info("Stage 1 complete. Waiting 3 seconds.")
    else:
        navigator.error("Stage 1 failed. Aborting.")
        return

    time.sleep(3.0)

    # --- 2단계 주행 ---
    set_crab_mode(navigator, True)
    time.sleep(1.0)

    navigator.info("Stage 2: Moving lateral (Crab mode).")
    navigator.goToPose(goal_2)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.info(f"Stage 2 - Distance remaining: {feedback.distance_remaining:.2f} m", throttle_duration_sec=1.0)
        time.sleep(0.1)

    if navigator.getResult() == TaskResult.SUCCEEDED:
        navigator.info("Stage 2 complete. Orthogonal docking successful.")
    else:
        navigator.error("Stage 2 failed.")

    set_crab_mode(navigator, False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()