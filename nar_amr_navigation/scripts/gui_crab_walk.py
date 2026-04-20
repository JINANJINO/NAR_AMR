#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import os
import threading
import tkinter as tk
from tkinter import messagebox

# 1. 파라미터 변경 함수
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

# 2. 주행 시퀀스 (무한 반복 가능하도록 수정)
def run_navigation_sequence(navigator, target_x, target_y, btn_start):
    try:
        navigator.info(f"New Task - Target X: {target_x}, Y: {target_y}")
        
        # [준비] 일반 모드로 시작
        set_crab_mode(navigator, False)
        time.sleep(0.5)

        # --- Stage 1: X축 이동 (직진) ---
        goal_1 = PoseStamped()
        goal_1.header.frame_id = 'map'
        goal_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_1.pose.position.x = float(target_x)
        goal_1.pose.position.y = 0.0  # 명확하게 0.0 지정
        goal_1.pose.orientation.w = 1.0

        navigator.info("Stage 1: Moving to X coordinate.")
        navigator.goToPose(goal_1)

        while not navigator.isTaskComplete():
            time.sleep(0.1)

        if navigator.getResult() != TaskResult.SUCCEEDED:
            navigator.error("Stage 1 failed.")
            btn_start.config(state=tk.NORMAL)
            return

        navigator.info("Stage 1 complete. Stabilizing...")
        time.sleep(2.0)

        # --- Stage 2: 게걸음 모드 변경 후 Y축 이동 ---
        set_crab_mode(navigator, True)
        time.sleep(1.0)

        goal_2 = PoseStamped()
        goal_2.header.frame_id = 'map'
        goal_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_2.pose.position.x = float(target_x) # X는 유지
        goal_2.pose.position.y = float(target_y) # Y 이동
        goal_2.pose.orientation.w = 1.0

        navigator.info("Stage 2: Crab walking to Y coordinate.")
        navigator.goToPose(goal_2)

        while not navigator.isTaskComplete():
            time.sleep(0.1)

        if navigator.getResult() == TaskResult.SUCCEEDED:
            navigator.info("Orthogonal docking successful.")
        else:
            navigator.error("Stage 2 failed.")

    except Exception as e:
        navigator.error(f"Error: {e}")
    finally:
        # 주행이 끝나면 다시 일반 모드로 복구하고 버튼 활성화
        set_crab_mode(navigator, False)
        btn_start.config(state=tk.NORMAL)

# 3. GUI 클래스
class AMRControlGUI:
    def __init__(self, root, navigator):
        self.root = root
        self.navigator = navigator
        self.root.title("Swerve AMR Control Panel")
        self.root.geometry("300x250")
        
        tk.Label(root, text="Enter Target Coordinates", font=("Helvetica", 12, "bold")).pack(pady=10)

        frame_x = tk.Frame(root); frame_x.pack(pady=2)
        tk.Label(frame_x, text="Target X (m):", width=12).pack(side=tk.LEFT)
        self.entry_x = tk.Entry(frame_x, width=10); self.entry_x.insert(0, "2.0"); self.entry_x.pack(side=tk.LEFT)

        frame_y = tk.Frame(root); frame_y.pack(pady=2)
        tk.Label(frame_y, text="Target Y (m):", width=12).pack(side=tk.LEFT)
        self.entry_y = tk.Entry(frame_y, width=10); self.entry_y.insert(0, "2.0"); self.entry_y.pack(side=tk.LEFT)

        self.btn_start = tk.Button(root, text="DOCKING START", bg="lightblue", command=self.on_start_clicked)
        self.btn_start.pack(pady=20)

    def on_start_clicked(self):
        try:
            tx = float(self.entry_x.get())
            ty = float(self.entry_y.get())
            self.btn_start.config(state=tk.DISABLED)
            # Daemon 스레드로 실행하여 GUI 종료 시 함께 종료되도록 함
            threading.Thread(target=run_navigation_sequence, args=(self.navigator, tx, ty, self.btn_start), daemon=True).start()
        except:
            messagebox.showerror("Error", "Please enter valid numbers.")

def main():
    rclpy.init()
    # Navigator 인스턴스를 하나만 생성해서 계속 재사용합니다.
    navigator = BasicNavigator()
    
    navigator.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()

    root = tk.Tk()
    app = AMRControlGUI(root, navigator)
    root.mainloop()

    # GUI 창을 완전히 닫았을 때만 rclpy를 종료합니다.
    rclpy.shutdown()

if __name__ == '__main__':
    main()