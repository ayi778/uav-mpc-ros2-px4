import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import casadi as ca
import math

import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import atexit

from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint, OffboardControlMode, VehicleCommand

class UavMpcNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_ctrl_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.state = np.zeros(6) 
        self.dt = 0.05            
        self.N = 30               
        
        self.setup_mpc()

        self.history_t = []
        self.history_x, self.history_y, self.history_z = [], [], []
        self.ref_x, self.ref_y, self.ref_z = [], [], []

        #atexit.register(self.plot_trajectory)

        self.offboard_counter = 0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Anti-Crash Figure-8 MPC Tracker Init. Press [Ctrl+C] to plot after 40s.")

    def setup_mpc(self):
        self.opti = ca.Opti()
        self.X = self.opti.variable(6, self.N + 1) 
        self.U = self.opti.variable(3, self.N)     
        self.P_ref = self.opti.parameter(6, self.N + 1) 
        self.X0 = self.opti.parameter(6)           

        A = np.eye(6)
        A[0:3, 3:6] = np.eye(3) * self.dt
        B = np.zeros((6, 3))
        B[0:3, 0:3] = 0.5 * np.eye(3) * self.dt**2
        B[3:6, 0:3] = np.eye(3) * self.dt

        # 🚀 修改这里：加大速度追踪的惩罚，放缓控制指令的锐度
        Q = np.diag([50.0, 50.0, 60.0, 5.0, 5.0, 2.0]) 
        R = np.diag([0.5, 0.5, 0.5])                   

        cost = 0
        self.opti.subject_to(self.X[:, 0] == self.X0) 

        for k in range(self.N):
            state_error = self.X[:, k] - self.P_ref[:, k]
            cost += ca.mtimes([state_error.T, Q, state_error]) + ca.mtimes([self.U[:, k].T, R, self.U[:, k]])
            x_next = ca.mtimes(A, self.X[:, k]) + ca.mtimes(B, self.U[:, k])
            self.opti.subject_to(self.X[:, k+1] == x_next)
            
            # 🚀 极其关键：大幅缩紧侧向加速度，最大倾角锁死在 17 度，彻底杜绝翻车！
            self.opti.subject_to(self.opti.bounded(-3.0, self.U[0, k], 3.0)) 
            self.opti.subject_to(self.opti.bounded(-3.0, self.U[1, k], 3.0)) 
            self.opti.subject_to(self.opti.bounded(-5.0, self.U[2, k], 2.0)) 

        self.opti.minimize(cost)
        p_opts = {"print_time": False}
        s_opts = {"print_level": 0, "sb": "yes"}
        self.opti.solver('ipopt', p_opts, s_opts)

    def odom_callback(self, msg):
        # 🚀 极其关键：锁定真实的出生原点，防止一出生就乱窜
        if not hasattr(self, 'initial_pos_set'):
            self.start_x = msg.position[0]
            self.start_y = msg.position[1]
            self.initial_pos_set = True
            self.get_logger().info(f"Home Locked: X={self.start_x:.2f}, Y={self.start_y:.2f}")

        self.state = np.array([
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2]
        ])

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def get_reference_trajectory(self, t_start):
        ref = np.zeros((6, self.N + 1))
        Radius = 4.0        
        omega = 0.2         
        z_target = -5.0     
        
        sx = getattr(self, 'start_x', 0.0)
        sy = getattr(self, 'start_y', 0.0)

        for i in range(self.N + 1):
            t = t_start + i * self.dt
            
            if t < 5.0:
                # 🚀 平滑起飞：前 5 秒内，Z 轴像电梯一样平滑上升，不准有猛烈机动
                current_z = z_target * (t / 5.0)
                vz = z_target / 5.0
                ref[0, i] = sx
                ref[1, i] = sy
                ref[2, i] = current_z
                ref[3, i] = 0.0
                ref[4, i] = 0.0
                ref[5, i] = vz
            else:
                # 5 秒后切入 8 字机动
                t_curve = t - 5.0  
                ref[0, i] = sx + Radius * math.sin(omega * t_curve)          
                ref[1, i] = sy + Radius * math.sin(2.0 * omega * t_curve)    
                ref[2, i] = z_target
                
                ref[3, i] = Radius * omega * math.cos(omega * t_curve)
                ref[4, i] = 2.0 * Radius * omega * math.cos(2.0 * omega * t_curve)
                ref[5, i] = 0.0                        
        return ref

    def control_loop(self):
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = True  
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_mode_pub.publish(mode_msg)

        current_t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        ref_traj = self.get_reference_trajectory(current_t)

        self.opti.set_value(self.X0, self.state)
        self.opti.set_value(self.P_ref, ref_traj)

        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.U[:, 0])
        except RuntimeError:
            u_opt = [0.0, 0.0, 0.0]

        setpoint_msg = TrajectorySetpoint()
        # 🚀 标准填充：规范化 NaN，强制锁死偏航角，防自旋坠机
        setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.acceleration = [float(u_opt[0]), float(u_opt[1]), float(u_opt[2])]
        setpoint_msg.yaw = 0.0 
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(setpoint_msg)

        if self.offboard_counter >= 20 and self.offboard_counter % 20 == 0 and self.offboard_counter < 200:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)
            self.get_logger().info("🚀 发射终极指令: 强制接管与解锁！(Retrying Arm...)")

        if self.offboard_counter >= 20:
            self.history_t.append(current_t)
            self.history_x.append(self.state[0])
            self.history_y.append(self.state[1])
            self.history_z.append(-self.state[2]) 
            self.ref_x.append(ref_traj[0, 0])
            self.ref_y.append(ref_traj[1, 0])
            self.ref_z.append(-ref_traj[2, 0])

        self.offboard_counter += 1

    def plot_trajectory(self):
        self.get_logger().info("Generating Figure-8 trajectory plot and calculating metrics...")
        if len(self.history_t) < 50:
            self.get_logger().warn("Not enough data points to calculate metrics.")
            return

        hx, hy, hz = np.array(self.history_x), np.array(self.history_y), np.array(self.history_z)
        rx, ry, rz = np.array(self.ref_x), np.array(self.ref_y), np.array(self.ref_z)
        
        # 1. 计算各个维度的绝对误差
        error_x = hx - rx
        error_y = hy - ry
        error_z = hz - rz
        
        # 2. 计算空间欧式距离误差
        error_distance = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # 3. 计算核心量化指标
        rmse_overall = np.sqrt(np.mean(error_distance**2))
        max_error = np.max(error_distance)
        mean_error = np.mean(error_distance)
        rmse_x = np.sqrt(np.mean(error_x**2))
        rmse_y = np.sqrt(np.mean(error_y**2))
        rmse_z = np.sqrt(np.mean(error_z**2))

        # 4. 在终端打印出极其专业的量化报告
        self.get_logger().info("\n" + "="*40)
        self.get_logger().info("🚀 实验量化误差结果 (Tracking Error Metrics)")
        self.get_logger().info("="*40)
        self.get_logger().info(f"总体均方根误差 (Overall RMSE) : {rmse_overall:.4f} m")
        self.get_logger().info(f"最大空间误差 (Max Error)      : {max_error:.4f} m")
        self.get_logger().info(f"平均空间误差 (Mean Error)     : {mean_error:.4f} m")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"X轴 RMSE (X-axis)             : {rmse_x:.4f} m")
        self.get_logger().info(f"Y轴 RMSE (Y-axis)             : {rmse_y:.4f} m")
        self.get_logger().info(f"Z轴 RMSE (Z-axis)             : {rmse_z:.4f} m")
        self.get_logger().info("="*40 + "\n")

        # 5. 将结果自动保存到 txt 文件，方便你写报告用
        with open('experiment_metrics.txt', 'w') as f:
            f.write("========== UAV MPC Tracking Metrics ==========\n")
            f.write(f"Overall RMSE : {rmse_overall:.4f} m\n")
            f.write(f"Max Error    : {max_error:.4f} m\n")
            f.write(f"Mean Error   : {mean_error:.4f} m\n")
            f.write(f"X-axis RMSE  : {rmse_x:.4f} m\n")
            f.write(f"Y-axis RMSE  : {rmse_y:.4f} m\n")
            f.write(f"Z-axis RMSE  : {rmse_z:.4f} m\n")
            f.write("==============================================\n")

        # 6. 继续生成图片 (把 RMSE 写在标题里)
        fig = plt.figure(figsize=(12, 5))
        
        ax1 = fig.add_subplot(1, 2, 1)
        ax1.plot(rx, ry, 'r--', label='Reference (Figure-8)')
        ax1.plot(hx, hy, 'b-', label='Actual Flight')
        ax1.set_title(f'XY Plane Top View')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

        ax2 = fig.add_subplot(1, 2, 2, projection='3d')
        ax2.plot(rx, ry, rz, 'r--', label='Reference')
        ax2.plot(hx, hy, hz, 'b-', label='Actual MPC')
        ax2.set_title(f'3D Trajectory (RMSE: {rmse_overall:.4f} m)')
        ax2.legend()

        plt.tight_layout()
        plt.savefig('figure8_result.png', dpi=300)
        self.get_logger().info("Plot saved as 'figure8_result.png'. Metrics saved to 'experiment_metrics.txt'.")

def main():
    rclpy.init()
    node = UavMpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 🚀 优雅退出：捕捉 Ctrl+C，在 ROS2 彻底关闭前安全地画图
        node.get_logger().info("\n⚠️ 收到中止信号，正在生成最终实验报告...")
        node.plot_trajectory()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()