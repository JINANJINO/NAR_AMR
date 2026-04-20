#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

# 각도를 -pi ~ pi 사이로 정규화
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# 두 각도 사이의 최단 거리 계산
def shortest_angular_distance(from_angle, to_angle):
    return normalize_angle(to_angle - from_angle)

class AdvancedSwerveController(Node):
    def __init__(self):
        super().__init__('sim_swerve_controller')
        self.get_logger().info("ROBOTIS FFW Logic Applied: Advanced Swerve Controller Started.")
        
        # 1. 구독 및 발행 설정
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # Gazebo의 실제 관절 상태를 읽어오기 위한 구독자
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pub_steer = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)

        # 2. 로봇 제원 (URDF 및 이전 코드 기준)
        self.L = 1.10    
        self.W = 0.88    
        self.r = 0.075   
        half_L, half_W = self.L / 2.0, self.W / 2.0
        
        # 모듈 순서: FL, FR, RL, RR (swerve_controllers.yaml 조인트 순서와 일치해야 함)
        self.modules = [
            (half_L, half_W),   # FL
            (half_L, -half_W),  # FR
            (-half_L, half_W),  # RL
            (-half_L, -half_W)  # RR
        ]

        self.ALIGNMENT_THRESHOLD = 0.15      
        self.MAX_ACCEL_LIN = 1.0             
        self.MAX_ACCEL_ANG = 2.0             
        
        # 4. 상태 변수
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # 실제 조인트 상태 저장용 변수
        self.actual_steer_angles = [0.0, 0.0, 0.0, 0.0]
        self.actual_wheel_speeds = [0.0, 0.0, 0.0, 0.0]

        # 오도메트리용
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0

        # 5. 제어 루프 (100Hz로 설정 - Gazebo의 부하를 줄이면서 충분히 부드러움)
        self.dt = 0.01 
        self.timer = self.create_timer(self.dt, self.control_loop)

    def cmd_cb(self, msg: Twist):
        """목표 속도 업데이트"""
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    def joint_cb(self, msg: JointState):
        """[NEW] Gazebo에서 실제 조향 각도와 바퀴 속도를 읽어옴"""
        # URDF 조인트 이름 기준 인덱스 매칭
        steer_names = ['fl_base_to_steering', 'fr_base_to_steering', 'rl_base_to_steering', 'rr_base_to_steering']
        drive_names = ['fl_steering_to_wheel', 'fr_steering_to_wheel', 'rl_steering_to_wheel', 'rr_steering_to_wheel']
        
        for i in range(4):
            try:
                # JointState 배열에서 해당 이름의 인덱스를 찾아 값 저장
                s_idx = msg.name.index(steer_names[i])
                d_idx = msg.name.index(drive_names[i])
                self.actual_steer_angles[i] = msg.position[s_idx]
                self.actual_wheel_speeds[i] = msg.velocity[d_idx]
            except ValueError:
                pass # 시작 직후 조인트 이름이 다 안들어왔을 때 예외처리

    def apply_speed_limiter(self):
        """[NEW] 가속도 제한 (Speed Limiter)"""
        def clamp(val, min_val, max_val):
            return max(min(val, max_val), min_val)

        # dt 동안 변할 수 있는 최대 속도량 계산
        dv_lin_max = self.MAX_ACCEL_LIN * self.dt
        dv_ang_max = self.MAX_ACCEL_ANG * self.dt

        self.current_vx = clamp(self.target_vx, self.current_vx - dv_lin_max, self.current_vx + dv_lin_max)
        self.current_vy = clamp(self.target_vy, self.current_vy - dv_lin_max, self.current_vy + dv_lin_max)
        self.current_wz = clamp(self.target_wz, self.current_wz - dv_ang_max, self.current_wz + dv_ang_max)

    def control_loop(self):
        """메인 제어 루프"""
        self.apply_speed_limiter()

        steer_commands = []
        drive_commands = []
        
        # 순기구학 (Forward Kinematics) 연산용 합계 변수
        est_vx_sum = 0.0
        est_vy_sum = 0.0

        for i, (mod_x, mod_y) in enumerate(self.modules):
            # 1. 역기구학 (Inverse Kinematics)
            vx_mod = self.current_vx - self.current_wz * mod_y
            vy_mod = self.current_vy + self.current_wz * mod_x
            
            ideal_angle = math.atan2(vy_mod, vx_mod)
            ideal_speed = math.hypot(vx_mod, vy_mod)

            actual_angle = self.actual_steer_angles[i]
            
            # 2. 조향 최적화 (Steering Optimization - Flip)
            # 현재 각도에서 가장 가까운 방향으로 돌림
            final_angle = ideal_angle
            final_speed = ideal_speed

            if ideal_speed > 0.001:
                # 이상적 각도와 현재 각도의 차이
                angle_diff = shortest_angular_distance(actual_angle, ideal_angle)
                
                # 90도(pi/2) 이상 돌아가야 한다면, 바퀴를 180도 뒤집고 역회전시키는 것이 빠름
                if abs(angle_diff) > math.pi / 2.0:
                    final_angle = normalize_angle(ideal_angle + math.pi)
                    final_speed = -ideal_speed
            else:
                # 정지 시 현재 조향각 유지
                final_angle = actual_angle
                final_speed = 0.0

            # 3. 조향 정렬 게이트 (Steering Alignment Gate)
            # 바퀴가 아직 목표 각도를 향해 도는 중이라면, 구동 속도를 0으로 만듦
            error = abs(shortest_angular_distance(actual_angle, final_angle))
            if error > self.ALIGNMENT_THRESHOLD and abs(final_speed) > 0.001:
                # 각도가 안 맞았으니 바퀴는 굴리지 마!
                drive_cmd = 0.0
            else:
                # 각도가 맞춰졌으니 굴러가도 좋아! (m/s -> rad/s 변환)
                drive_cmd = final_speed / self.r

            steer_commands.append(final_angle)
            drive_commands.append(drive_cmd)

            # 4. 폐루프 기반 순기구학 (Forward Kinematics)
            # 우리가 명령한 값이 아니라, '실제' 조향각과 '실제' 바퀴 속도로 로봇의 이동량 추정
            actual_lin_vel = self.actual_wheel_speeds[i] * self.r
            est_vx_sum += actual_lin_vel * math.cos(actual_angle)
            est_vy_sum += actual_lin_vel * math.sin(actual_angle)

        # 명령 발행
        self.pub_steer.publish(Float64MultiArray(data=steer_commands))
        self.pub_drive.publish(Float64MultiArray(data=drive_commands))

        # 5. 오도메트리 계산 및 퍼블리시
        avg_vx = est_vx_sum / 4.0
        avg_vy = est_vy_sum / 4.0
        # Yaw 변화율은 명령된 각속도 사용 (또는 IMU 데이터와 EKF에서 융합)
        avg_vth = self.current_wz 

        # 로컬(Robot) -> 글로벌(Odom) 프레임 변환 적분
        self.odom_x += (avg_vx * math.cos(self.odom_th) - avg_vy * math.sin(self.odom_th)) * self.dt
        self.odom_y += (avg_vx * math.sin(self.odom_th) + avg_vy * math.cos(self.odom_th)) * self.dt
        self.odom_th += avg_vth * self.dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = math.sin(self.odom_th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.odom_th / 2.0)
        
        # 공분산
        odom.pose.covariance[0] = 0.01  
        odom.pose.covariance[7] = 0.01  
        odom.pose.covariance[35] = 0.05 
        
        odom.twist.twist.linear.x = avg_vx
        odom.twist.twist.linear.y = avg_vy
        odom.twist.twist.angular.z = avg_vth
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedSwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()