import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
import threading

class SwerveGUI(Node):
    def __init__(self):
        super().__init__('swerve_gui_controller')
        
        # 퍼블리셔 생성
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)

        # GUI 창 설정
        self.root = tk.Tk()
        self.root.title("NeoArc Swerve Controller")
        self.root.geometry("300x400")

        # 버튼 생성 함수
        self.create_buttons()

    def create_buttons(self):
        tk.Label(self.root, text="Swerve Drive Control", font=("Arial", 16, "bold")).pack(pady=10)

        # [조향 각도], [구동 속도]
        btn_fw = tk.Button(self.root, text="⬆️ 직진", font=("Arial", 14), command=lambda: self.send_cmd([0.0, 0.0, 0.0, 0.0], [5.0, 5.0, 5.0, 5.0]))
        btn_fw.pack(fill='x', padx=20, pady=5)

        btn_bw = tk.Button(self.root, text="⬇️ 후진", font=("Arial", 14), command=lambda: self.send_cmd([0.0, 0.0, 0.0, 0.0], [-5.0, -5.0, -5.0, -5.0]))
        btn_bw.pack(fill='x', padx=20, pady=5)

        btn_left = tk.Button(self.root, text="⬅️ 좌측 게걸음", font=("Arial", 14), command=lambda: self.send_cmd([1.57, 1.57, 1.57, 1.57], [5.0, 5.0, 5.0, 5.0]))
        btn_left.pack(fill='x', padx=20, pady=5)

        btn_right = tk.Button(self.root, text="➡️ 우측 게걸음", font=("Arial", 14), command=lambda: self.send_cmd([1.57, 1.57, 1.57, 1.57], [-5.0, -5.0, -5.0, -5.0]))
        btn_right.pack(fill='x', padx=20, pady=5)

        btn_cw = tk.Button(self.root, text="↻ 제자리 우회전", font=("Arial", 14), command=lambda: self.send_cmd([-0.785, 0.785, 0.785, -0.785], [5.0, -5.0, 5.0, -5.0]))
        btn_cw.pack(fill='x', padx=20, pady=5)

        btn_ccw = tk.Button(self.root, text="↺ 제자리 좌회전", font=("Arial", 14), command=lambda: self.send_cmd([-0.785, 0.785, 0.785, -0.785], [-5.0, 5.0, -5.0, 5.0]))
        btn_ccw.pack(fill='x', padx=20, pady=5)

        btn_stop = tk.Button(self.root, text="🛑 정지 (STOP)", font=("Arial", 14, "bold"), bg="red", fg="white", command=lambda: self.send_cmd([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]))
        btn_stop.pack(fill='x', padx=20, pady=15)

    def send_cmd(self, steer_angles, drive_vels):
        # 1. 조향 명령 먼저 보내기 (방향 정렬)
        msg_steer = Float64MultiArray()
        msg_steer.data = steer_angles
        self.steer_pub.publish(msg_steer)

        # 2. 약간의 딜레이 후 구동 명령 보내기 (자연스러운 주행을 위해)
        def drive():
            msg_drive = Float64MultiArray()
            msg_drive.data = drive_vels
            self.drive_pub.publish(msg_drive)
        
        self.root.after(100, drive) # 0.1초 뒤 바퀴 굴림
        self.get_logger().info(f"CMD Sent -> Steer: {steer_angles}, Drive: {drive_vels}")

def main(args=None):
    rclpy.init(args=args)
    node = SwerveGUI()

    # ROS 2 Spin을 별도 스레드로 실행 (GUI 멈춤 방지)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # GUI 실행
    node.root.mainloop()

    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()