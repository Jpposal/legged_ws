#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque

# --- 配置参数 ---
UPDATE_INTERVAL = 100        # 动画刷新间隔 (ms)
MAX_HISTORY = 2000           # 最大保存数据点数 (防止绘图卡死)
OBJECT_NAME = "shared_block" 

# Topics
TOPIC_CoM_ACTUAL = "/shared_object/state"
TOPIC_CoM_TARGET = "/arm_leader_controller/command_pose"

TOPIC_LEADER_ACTUAL = "/leader_actual_pose"
TOPIC_LEADER_DESIRED = "/leader_des_ee"

TOPIC_FOLLOWER_ACTUAL = "/follower_actual_pose"
TOPIC_FOLLOWER_DESIRED = "/follower_des_ee"

TOPIC_LEADER_PARAMS = "/leader_drem_params"

class MultiTrackingPlotter:
    def __init__(self):
        rospy.init_node('multi_tracking_plotter', anonymous=True)
        
        self.start_time = rospy.Time.now().to_sec()
        
        # --- Data Stores (Using deque for rolling buffer) ---
        # CoM
        self.data_com = {
            't_act': deque(maxlen=MAX_HISTORY), 'x_act': deque(maxlen=MAX_HISTORY), 'y_act': deque(maxlen=MAX_HISTORY), 'z_act': deque(maxlen=MAX_HISTORY),
            't_tgt': deque(maxlen=MAX_HISTORY), 'x_tgt': deque(maxlen=MAX_HISTORY), 'y_tgt': deque(maxlen=MAX_HISTORY), 'z_tgt': deque(maxlen=MAX_HISTORY)
        }
        # Leader
        self.data_leader = {
            't_act': deque(maxlen=MAX_HISTORY), 'x_act': deque(maxlen=MAX_HISTORY), 'y_act': deque(maxlen=MAX_HISTORY), 'z_act': deque(maxlen=MAX_HISTORY),
            't_des': deque(maxlen=MAX_HISTORY), 'x_des': deque(maxlen=MAX_HISTORY), 'y_des': deque(maxlen=MAX_HISTORY), 'z_des': deque(maxlen=MAX_HISTORY)
        }
        # Follower
        self.data_follower = {
            't_act': deque(maxlen=MAX_HISTORY), 'x_act': deque(maxlen=MAX_HISTORY), 'y_act': deque(maxlen=MAX_HISTORY), 'z_act': deque(maxlen=MAX_HISTORY),
            't_des': deque(maxlen=MAX_HISTORY), 'x_des': deque(maxlen=MAX_HISTORY), 'y_des': deque(maxlen=MAX_HISTORY), 'z_des': deque(maxlen=MAX_HISTORY)
        }
        
        # Leader Mass Est
        self.data_mass = {
            't': deque(maxlen=MAX_HISTORY), 
            'm_est': deque(maxlen=MAX_HISTORY)
        }

        # --- Subscribers ---
        # CoM
        rospy.Subscriber(TOPIC_CoM_ACTUAL, Odometry, self.cb_com_actual)
        rospy.Subscriber(TOPIC_CoM_TARGET, PoseStamped, self.cb_com_target)
        
        # Leader
        rospy.Subscriber(TOPIC_LEADER_ACTUAL, PoseStamped, self.cb_leader_actual)
        rospy.Subscriber(TOPIC_LEADER_DESIRED, Vector3Stamped, self.cb_leader_desired)
        rospy.Subscriber(TOPIC_LEADER_PARAMS, Float64MultiArray, self.cb_leader_params)
        
        # Follower
        rospy.Subscriber(TOPIC_FOLLOWER_ACTUAL, PoseStamped, self.cb_follower_actual)
        rospy.Subscriber(TOPIC_FOLLOWER_DESIRED, Vector3Stamped, self.cb_follower_desired)
        
        # --- Plot Layout ---
        # 3 Rows: X, Y, Z
        # 3 Columns: CoM, Leader, Follower
        self.fig, self.axs = plt.subplots(3, 3, sharex=True, figsize=(15, 10))
        self.fig.suptitle('Multi-Body Tracking Dashboard', fontsize=16)
        
        # Set Title for first row (Columns)
        self.axs[0, 0].set_title(f"Object CoM ({OBJECT_NAME})")
        self.axs[0, 1].set_title("Leader EE")
        self.axs[0, 2].set_title("Follower EE")
        
        # Initialize Lines Dictionary
        # Key: (row, col, type)
        # row: 0=X, 1=Y, 2=Z
        # col: 0=CoM, 1=Leader, 2=Follower
        # type: 'act' or 'des'
        self.lines = {} 
        
        self.setup_column(0, "CoM")
        self.setup_column(1, "Leader")
        self.setup_column(2, "Follower")
        
        plt.tight_layout()
        
        # --- Figure 2: Parameter Estimation ---
        self.fig2, self.ax_mass = plt.subplots(figsize=(6, 4))
        self.fig2.canvas.manager.set_window_title('Parameter Estimation')
        self.ax_mass.set_title('Estimated Object Mass (Leader)')
        self.ax_mass.set_ylabel('Mass (kg)')
        self.ax_mass.set_xlabel('Time (s)')
        self.ax_mass.grid(True)
        self.line_mass, = self.ax_mass.plot([], [], 'g-', linewidth=2, label='Est. Mass')
        self.ax_mass.legend()

        # Animation (Update both figures)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=UPDATE_INTERVAL)
        self.ani2 = animation.FuncAnimation(self.fig2, self.update_mass_plot, interval=UPDATE_INTERVAL)
        
        rospy.loginfo("MultiTrackingPlotter Initialized.")
        plt.show()

    def setup_column(self, col_idx, label_prefix):
        props_act = {'color': 'blue', 'linestyle': '-', 'linewidth': 2, 'label': 'Actual'}
        props_des = {'color': 'red', 'linestyle': '--', 'linewidth': 2, 'label': 'Target/Des'}
        
        # X Row (Row 0)
        ax_x = self.axs[0, col_idx]
        ax_x.set_ylabel('X (m)')
        l_act_x, = ax_x.plot([], [], **props_act)
        l_des_x, = ax_x.plot([], [], **props_des)
        ax_x.grid(True)
        # Only legend on first cell
        if col_idx == 0: ax_x.legend(loc='upper right', fontsize='small')
        
        # Y Row (Row 1)
        ax_y = self.axs[1, col_idx]
        ax_y.set_ylabel('Y (m)')
        l_act_y, = ax_y.plot([], [], **props_act)
        l_des_y, = ax_y.plot([], [], **props_des)
        ax_y.grid(True)

        # Z Row (Row 2)
        ax_z = self.axs[2, col_idx]
        ax_z.set_ylabel('Z (m)')
        ax_z.set_xlabel('Time (s)')
        l_act_z, = ax_z.plot([], [], **props_act)
        l_des_z, = ax_z.plot([], [], **props_des)
        ax_z.grid(True)
        
        # Store refs
        self.lines[(0, col_idx, 'act')] = l_act_x
        self.lines[(0, col_idx, 'des')] = l_des_x
        self.lines[(1, col_idx, 'act')] = l_act_y
        self.lines[(1, col_idx, 'des')] = l_des_y
        self.lines[(2, col_idx, 'act')] = l_act_z
        self.lines[(2, col_idx, 'des')] = l_des_z

    # --- Callbacks ---
    def get_time(self):
        return rospy.Time.now().to_sec() - self.start_time

    # CoM
    def cb_com_actual(self, msg):
        t = self.get_time()
        self.data_com['t_act'].append(t)
        self.data_com['x_act'].append(msg.pose.pose.position.x)
        self.data_com['y_act'].append(msg.pose.pose.position.y)
        self.data_com['z_act'].append(msg.pose.pose.position.z)

    def cb_com_target(self, msg):
        t = self.get_time()
        self.data_com['t_tgt'].append(t)
        self.data_com['x_tgt'].append(msg.pose.position.x)
        self.data_com['y_tgt'].append(msg.pose.position.y)
        self.data_com['z_tgt'].append(msg.pose.position.z)

    # Leader
    def cb_leader_actual(self, msg):
        t = self.get_time()
        self.data_leader['t_act'].append(t)
        self.data_leader['x_act'].append(msg.pose.position.x)
        self.data_leader['y_act'].append(msg.pose.position.y)
        self.data_leader['z_act'].append(msg.pose.position.z)

    def cb_leader_desired(self, msg):
        t = self.get_time()
        self.data_leader['t_des'].append(t)
        self.data_leader['x_des'].append(msg.vector.x)
        self.data_leader['y_des'].append(msg.vector.y)
        self.data_leader['z_des'].append(msg.vector.z)

    # Follower
    def cb_follower_actual(self, msg):
        t = self.get_time()
        self.data_follower['t_act'].append(t)
        self.data_follower['x_act'].append(msg.pose.position.x)
        self.data_follower['y_act'].append(msg.pose.position.y)
        self.data_follower['z_act'].append(msg.pose.position.z)

    def cb_follower_desired(self, msg):
        t = self.get_time()
        self.data_follower['t_des'].append(t)
        self.data_follower['x_des'].append(msg.vector.x)
        self.data_follower['y_des'].append(msg.vector.y)
        self.data_follower['z_des'].append(msg.vector.z)

    def cb_leader_params(self, msg):
        if len(msg.data) > 0:
            t = self.get_time()
            self.data_mass['t'].append(t)
            self.data_mass['m_est'].append(msg.data[0]) # Mass is index 0

    def update_mass_plot(self, frame):
        if self.data_mass['t']:
            self.line_mass.set_data(self.data_mass['t'], self.data_mass['m_est'])
            self.ax_mass.relim()
            self.ax_mass.autoscale_view()

    def update_plot(self, frame):
        # Update CoM (Column 0)
        self.update_column(0, self.data_com, 'tgt')
        # Update Leader (Column 1)
        self.update_column(1, self.data_leader, 'des')
        # Update Follower (Column 2)
        self.update_column(2, self.data_follower, 'des')
        
        # Use simple list conversion for determining limits
        all_times = list(self.data_com['t_act']) + list(self.data_leader['t_act']) + list(self.data_follower['t_act'])
            
        if all_times:
             t_max = max(all_times)
             t_min = min(all_times)
             # Set X limits for bottom row (shared) - Rolling Window View
             self.axs[2, 0].set_xlim(t_min, t_max + 0.1)
             
             # Auto-scale Y for all subplots
             for ax_row in self.axs:
                 for ax in ax_row:
                     ax.relim()
                     ax.autoscale_view(scalex=False, scaley=True)

    def update_column(self, col_idx, data, des_key_suffix):
        # Actual
        self.lines[(0, col_idx, 'act')].set_data(data['t_act'], data['x_act'])
        self.lines[(1, col_idx, 'act')].set_data(data['t_act'], data['y_act'])
        self.lines[(2, col_idx, 'act')].set_data(data['t_act'], data['z_act'])
        
        # Desired/Target
        t_des = data[f't_{des_key_suffix}'] # t_des is a deque
        self.lines[(0, col_idx, 'des')].set_data(t_des, data[f'x_{des_key_suffix}'])
        self.lines[(1, col_idx, 'des')].set_data(t_des, data[f'y_{des_key_suffix}'])
        self.lines[(2, col_idx, 'des')].set_data(t_des, data[f'z_{des_key_suffix}'])

if __name__ == '__main__':
    try:
        MultiTrackingPlotter()
    except rospy.ROSInterruptException:
        pass
