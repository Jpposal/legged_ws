#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# --- 配置参数 ---
UPDATE_INTERVAL = 100        # 动画刷新间隔 (ms)
OBJECT_NAME = "shared_block" # Gazebo中被抓物体的名称 (质心)
TARGET_TOPIC = "/arm_leader_controller/command_pose" # 目标轨迹话题

class CoMTrackingPlotter:
    def __init__(self):
        rospy.init_node('com_tracking_plotter', anonymous=True)
        
        self.start_time = rospy.Time.now().to_sec()
        
        # --- 数据存储 (使用列表存储全量数据) ---
        # 实际值 (Actual from Gazebo)
        self.t_act = []
        self.x_act = []
        self.y_act = []
        self.z_act = []
        
        # 目标值 (Target from Controller Command)
        self.t_tgt = []
        self.x_tgt = []
        self.y_tgt = []
        self.z_tgt = []
        
        # --- 订阅话题 ---
        # 1. 获取物体真实质心位姿 (Modified to use specific ground truth topic)
        rospy.Subscriber("/shared_object/state", Odometry, self.actual_cb)
        # 2. 获取控制器发出的质心目标位姿
        rospy.Subscriber(TARGET_TOPIC, PoseStamped, self.target_cb)
        
        # --- 绘图初始化 ---
        self.fig, (self.ax_x, self.ax_y, self.ax_z) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        self.fig.suptitle(f'Object CoM Tracking: Actual({OBJECT_NAME}) vs Target', fontsize=14)
        
        self.setup_axes()
        
        # 启动动画循环
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=UPDATE_INTERVAL)
        
        rospy.loginfo(f"Plotter initialized. Monitoring '{OBJECT_NAME}' on /gazebo/model_states and target on {TARGET_TOPIC}")
        plt.show()

    def setup_axes(self):
        # 通用设置
        lines_props = {'lw': 2}
        
        # X Axis
        self.ax_x.set_ylabel('X Position (m)')
        self.ln_act_x, = self.ax_x.plot([], [], 'b-', label='Actual (Gazebo)', **lines_props)
        self.ln_tgt_x, = self.ax_x.plot([], [], 'r--', label='Target (Command)', **lines_props)
        self.ax_x.grid(True)
        self.ax_x.legend(loc='upper right')
        
        # Y Axis
        self.ax_y.set_ylabel('Y Position (m)')
        self.ln_act_y, = self.ax_y.plot([], [], 'b-', **lines_props)
        self.ln_tgt_y, = self.ax_y.plot([], [], 'r--', **lines_props)
        self.ax_y.grid(True)
        
        # Z Axis
        self.ax_z.set_ylabel('Z Position (m)')
        self.ax_z.set_xlabel('Time (s)')
        self.ln_act_z, = self.ax_z.plot([], [], 'b-', **lines_props)
        self.ln_tgt_z, = self.ax_z.plot([], [], 'r--', **lines_props)
        self.ax_z.grid(True)

    def actual_cb(self, msg):
        """处理 Gazebo 真实状态回调 (via Odometry)"""
        pose = msg.pose.pose
        
        t = rospy.Time.now().to_sec() - self.start_time
        self.t_act.append(t)
        self.x_act.append(pose.position.x)
        self.y_act.append(pose.position.y)
        self.z_act.append(pose.position.z)

    def target_cb(self, msg):
        """处理目标指令回调"""
        t = rospy.Time.now().to_sec() - self.start_time
        self.t_tgt.append(t)
        self.x_tgt.append(msg.pose.position.x)
        self.y_tgt.append(msg.pose.position.y)
        self.z_tgt.append(msg.pose.position.z)

    def update_plot(self, frame):
        """动画更新函数"""
        # 更新 X 数据
        self.ln_act_x.set_data(self.t_act, self.x_act)
        self.ln_tgt_x.set_data(self.t_tgt, self.x_tgt)
        
        # 更新 Y 数据
        self.ln_act_y.set_data(self.t_act, self.y_act)
        self.ln_tgt_y.set_data(self.t_tgt, self.y_tgt)
        
        # 更新 Z 数据
        self.ln_act_z.set_data(self.t_act, self.z_act)
        self.ln_tgt_z.set_data(self.t_tgt, self.z_tgt)
        
        # 自动调整坐标轴范围
        if self.t_act or self.t_tgt:
            # 获取当前所有时间数据的最大最小值
            all_t = self.t_act + self.t_tgt
            if all_t:
                self.ax_z.set_xlim(min(all_t), max(all_t) + 0.1)
                
            self.ax_x.relim()
            self.ax_x.autoscale_view(scalex=False, scaley=True)
            
            self.ax_y.relim()
            self.ax_y.autoscale_view(scalex=False, scaley=True)
            
            self.ax_z.relim()
            self.ax_z.autoscale_view(scalex=False, scaley=True)
            
        return self.ln_act_x, self.ln_tgt_x, self.ln_act_y, self.ln_tgt_y, self.ln_act_z, self.ln_tgt_z

if __name__ == '__main__':
    try:
        CoMTrackingPlotter()
    except rospy.ROSInterruptException:
        pass
