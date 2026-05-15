#!/usr/bin/env python3
########################################################################
#  Author: Jinhan Lee
#  Date: 2026-04-13
#  Description: NeoArc Swerve Advanced Remote GUI for ROS2 Jazzy
########################################################################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import math

CMD_VEL_TOPIC = '/cmd_vel'
MANUAL_STEER_OVERRIDE_TOPIC = '/manual_steer_override'
# integrated_swerve_controller는 조향 게이트 등으로 연속 cmd_vel에 가깝게 동작하는 경우가 많음
CMD_STREAM_HZ = 20.0


class AdvancedTwistGUI(Node):
    def __init__(self):
        super().__init__('swerve_advanced_gui')
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.steer_override_pub = self.create_publisher(Float32MultiArray, MANUAL_STEER_OVERRIDE_TOPIC, 10)
        self.get_logger().info(f'cmd_vel publish topic: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'manual steer publish topic: {MANUAL_STEER_OVERRIDE_TOPIC}')

        # Teleop Parameters
        self.lin_vel = 0.5
        self.ang_vel = 0.5
        self.lin_step = 0.1
        self.ang_step = 0.1
        self.dir_x, self.dir_y, self.dir_z = 0.0, 0.0, 0.0
        self._cmd_lock = threading.Lock()
        self._motion_active = False
        period = 1.0 / CMD_STREAM_HZ
        self._cmd_stream_timer = self.create_timer(period, self._cmd_stream_tick)
        self.get_logger().info(
            f'cmd_vel stream: {CMD_STREAM_HZ:.0f} Hz while a direction button is held.'
        )

        # Individual Steer Angles (Degree)
        self.steer_angles = [0.0, 0.0, 0.0, 0.0] # FL, FR, RL, RR
        self.wheel_labels = ['FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)']

        self.setup_ui()

    def setup_ui(self):
        self.root = tk.Tk()
        self.root.title("NeoArc Swerve Control Panel")
        self.root.geometry("600x650")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TButton', font=('Helvetica', 11), padding=5)
        style.configure('Header.TLabel', font=('Helvetica', 14, 'bold'))
        style.configure('Status.TLabel', font=('Helvetica', 12), foreground='#0052cc')

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        notebook.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        # 탭 1: Teleop
        teleop_frame = ttk.Frame(notebook)
        notebook.add(teleop_frame, text=' 🕹️ Teleop Control ')
        self.build_teleop_tab(teleop_frame)

        # 탭 2: 개별 조향 (Calibration)
        steer_frame = ttk.Frame(notebook)
        notebook.add(steer_frame, text=' ⚙️ Steer Calibration ')
        self.build_steer_tab(steer_frame)

    def _on_tab_changed(self, _event=None) -> None:
        self.publish_stop()

    def _add_motion_button(self, parent, text, dx, dy, dz, **grid_kw) -> ttk.Button:
        btn = ttk.Button(parent, text=text)
        btn.grid(**grid_kw)
        btn.bind('<ButtonPress-1>', lambda _e: self._on_motion_press(dx, dy, dz))
        btn.bind('<ButtonRelease-1>', lambda _e: self._on_motion_release())
        btn.bind('<Leave>', lambda _e: self._on_motion_release())
        return btn

    def build_teleop_tab(self, parent):
        ttk.Label(parent, text="AMR Movement (cmd_vel)", style='Header.TLabel').pack(pady=10)

        self.lbl_speed = ttk.Label(parent, text=self.get_speed_text(), style='Status.TLabel')
        self.lbl_speed.pack(pady=5)

        speed_frame = ttk.Frame(parent)
        speed_frame.pack(pady=10)

        ttk.Button(speed_frame, text="+ Lin Vel", command=self.inc_lin).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(speed_frame, text="- Lin Vel", command=self.dec_lin).grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(speed_frame, text="+ Ang Vel", command=self.inc_ang).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(speed_frame, text="- Ang Vel", command=self.dec_ang).grid(row=1, column=1, padx=5, pady=2)

        ctrl_frame = ttk.Frame(parent)
        ctrl_frame.pack(pady=15)

        self._add_motion_button(ctrl_frame, "↖", 1, 1, 0, row=0, column=0, padx=2, pady=2)
        self._add_motion_button(ctrl_frame, "⬆️ Straight", 1, 0, 0, row=0, column=1, padx=2, pady=2, sticky='ew')
        self._add_motion_button(ctrl_frame, "↗", 1, -1, 0, row=0, column=2, padx=2, pady=2)
        self._add_motion_button(ctrl_frame, "⬅️ Left", 0, 1, 0, row=1, column=0, padx=2, pady=2, sticky='ew')
        stop_btn = tk.Button(ctrl_frame, text="🛑 STOP", font=("Helvetica", 12, "bold"), bg="#d9534f", fg="white", command=self.publish_stop)
        stop_btn.grid(row=1, column=1, padx=5, pady=5, sticky='ew', ipadx=10, ipady=10)
        self._add_motion_button(ctrl_frame, "➡️ Right", 0, -1, 0, row=1, column=2, padx=2, pady=2, sticky='ew')
        self._add_motion_button(ctrl_frame, "↺ CCW", 0, 0, 1, row=2, column=0, padx=2, pady=2, sticky='ew')
        self._add_motion_button(ctrl_frame, "⬇️ Backward", -1, 0, 0, row=2, column=1, padx=2, pady=2, sticky='ew')
        self._add_motion_button(ctrl_frame, "↻ CW", 0, 0, -1, row=2, column=2, padx=2, pady=2, sticky='ew')


    def build_steer_tab(self, parent):
        ttk.Label(parent, text="Individual Wheel Alignment (Degrees)", style='Header.TLabel').pack(pady=15)
        ttk.Label(parent, text="※ warning: Adjusting these values may affect vehicle performance.", foreground="red").pack(pady=5)

        self.angle_labels = []
        for i in range(4):
            frame = ttk.Frame(parent)
            frame.pack(fill='x', padx=50, pady=10)
            
            ttk.Label(frame, text=self.wheel_labels[i], width=15).pack(side='left')
            
            ttk.Button(frame, text="-1°", width=4, command=lambda idx=i: self.adjust_steer(idx, -1.0)).pack(side='left', padx=5)
            
            lbl = ttk.Label(frame, text=f"{self.steer_angles[i]:.1f}°", font=('Helvetica', 12, 'bold'), width=8, anchor='center')
            lbl.pack(side='left', padx=10)
            self.angle_labels.append(lbl)
            
            ttk.Button(frame, text="+1°", width=4, command=lambda idx=i: self.adjust_steer(idx, 1.0)).pack(side='left', padx=5)

        btn_frame = ttk.Frame(parent)
        btn_frame.pack(pady=30)
        
        ttk.Button(btn_frame, text="Zero All (0°)", command=self.zero_all_steer).pack(side='left', padx=10)
        tk.Button(btn_frame, text="PUBLISH OVERRIDE", font=("Helvetica", 11, "bold"), bg="#5cb85c", fg="white", command=self.publish_steer_override).pack(side='left', padx=10, ipadx=10, ipady=5)

    def get_speed_text(self):
        return f"Linear: {self.lin_vel:.1f} m/s | Angular: {self.ang_vel:.1f} rad/s"

    def update_speed_label(self):
        self.lbl_speed.config(text=self.get_speed_text())

    def _build_twist_from_dirs(self) -> Twist:
        msg = Twist()
        with self._cmd_lock:
            msg.linear.x = self.dir_x * self.lin_vel
            msg.linear.y = self.dir_y * self.lin_vel
            msg.angular.z = self.dir_z * self.ang_vel
        return msg

    def _cmd_stream_tick(self) -> None:
        with self._cmd_lock:
            active = self._motion_active
        if not active:
            return
        self.cmd_pub.publish(self._build_twist_from_dirs())

    def _on_motion_press(self, dx, dy, dz) -> None:
        fx, fy, fz = float(dx), float(dy), float(dz)
        with self._cmd_lock:
            self.dir_x, self.dir_y, self.dir_z = fx, fy, fz
            self._motion_active = True
        msg = self._build_twist_from_dirs()
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"CMD press: vx={msg.linear.x:.1f}, vy={msg.linear.y:.1f}, wz={msg.angular.z:.1f}")

    def _on_motion_release(self) -> None:
        with self._cmd_lock:
            if not self._motion_active:
                return
        self.publish_stop()

    def inc_lin(self):
        with self._cmd_lock:
            self.lin_vel += self.lin_step
        self.update_speed_label()
        self._republish_if_moving()

    def dec_lin(self):
        with self._cmd_lock:
            self.lin_vel = max(0.0, self.lin_vel - self.lin_step)
        self.update_speed_label()
        self._republish_if_moving()

    def inc_ang(self):
        with self._cmd_lock:
            self.ang_vel += self.ang_step
        self.update_speed_label()
        self._republish_if_moving()

    def dec_ang(self):
        with self._cmd_lock:
            self.ang_vel = max(0.0, self.ang_vel - self.ang_step)
        self.update_speed_label()
        self._republish_if_moving()

    def _republish_if_moving(self) -> None:
        with self._cmd_lock:
            if not self._motion_active:
                return
        self.cmd_pub.publish(self._build_twist_from_dirs())

    # Steer Calibration Logic
    def adjust_steer(self, index, amount):
        self.steer_angles[index] += amount
        self.angle_labels[index].config(text=f"{self.steer_angles[index]:.1f}°")

    def zero_all_steer(self):
        self.steer_angles = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            self.angle_labels[i].config(text="0.0°")

    def publish_steer_override(self):
        self.publish_stop()
        msg = Float32MultiArray()
        # 컨트롤러는 Radian을 사용하므로 변환하여 전송
        msg.data = [math.radians(a) for a in self.steer_angles]
        self.steer_override_pub.publish(msg)
        self.get_logger().info(f"Steer Override Sent: {self.steer_angles} degrees")


    def publish_stop(self):
        with self._cmd_lock:
            self._motion_active = False
            self.dir_x, self.dir_y, self.dir_z = 0.0, 0.0, 0.0
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.get_logger().info("STOP Sent: vx=0.0, vy=0.0, wz=0.0")

    def on_close(self):
        try:
            self.publish_stop()
        finally:
            self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTwistGUI()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()