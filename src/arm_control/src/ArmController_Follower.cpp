#include "arm_control/ArmController_Follower.h"
#include <my_robot_controllers/robot_follower.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace arm_control {

bool ArmController_Follower::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  nh_ = nh;

  //这里是读取通过dual_arm_controllers.yaml加载的参数
  //dual_arm_controllers.yaml加载的参数是根据自己设定的
  std::string urdfFile, eeFrame;
  nh.param<std::string>("urdf_file", urdfFile, "");
  nh.param<std::string>("ee_frame", eeFrame, "tool0");
  nh.param<double>("kp", kp_, 100.0);
  nh.param<double>("kd", kd_, 10.0);
  
  double bx, by, bz;
  nh.param<double>("base_pos_x", bx, 0.0);
  nh.param<double>("base_pos_y", by, 0.0);
  nh.param<double>("base_pos_z", bz, 0.8);
  basePos_ << bx, by, bz;

  double ax, ay, az;
  nh.param<double>("anchor_offset_x", ax, 0.6);
  nh.param<double>("anchor_offset_y", ay, 0.0);
  nh.param<double>("anchor_offset_z", az, 0.15);
  anchor_offset_ << ax, ay, az;
  ROS_INFO_STREAM("Follower Anchor Offset (EE->CoM): " << anchor_offset_.transpose());

  //还是读取通过dual_arm_controllers.yaml加载的参数
  if (urdfFile.empty()) {
    std::string robotDescriptionParam;
    if (nh_.getParam("robot_description_param", robotDescriptionParam)) {
       ROS_INFO_STREAM("Using robot_description_param: " << robotDescriptionParam);
    } else {
       ROS_ERROR("Failed to find 'robot_description_param' in controller configuration. Please set it in your .yaml file.");
       return false;
    }
  //读取单机械臂的urdf描述
    std::string urdfString;
    if (nh_.getParam(robotDescriptionParam, urdfString)) {
       ROS_INFO_STREAM("Found URDF string at parameter: " << robotDescriptionParam);
    } else {
       ROS_ERROR_STREAM("Failed to find URDF string at parameter: " << robotDescriptionParam);
       return false;
    }
  //缓存单机械臂的urdf描述到临时文件
    if (!urdfString.empty()) {
       std::string ns = nh_.getNamespace();
       std::replace(ns.begin(), ns.end(), '/', '_');
       urdfFile = "/tmp/temp_arm_control_follower" + ns + ".urdf";
       std::ofstream out(urdfFile);
       out << urdfString;
       out.close();
       ROS_INFO_STREAM("Written robot_description to " << urdfFile);
    } else {
       ROS_ERROR_STREAM("URDF string is empty at parameter: " << robotDescriptionParam);
       return false;
    }
  }
  
  //读取yaml里面需要控制的电机（关节）列表
  if (nh.getParam("joint_names", jointNames_)) {
     ROS_INFO_STREAM("Found joint_names param, size=" << jointNames_.size());
  } else {
     ROS_ERROR("Failed to find 'joint_names' parameter. It must be specified in the controller yaml config.");
     return false;
  }

  //Pinocchio加载urdf模型
  try {
    pinocchio::urdf::buildModel(urdfFile, pinocchioModel_);
    pinocchioData_ = std::make_unique<pinocchio::Data>(pinocchioModel_);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to load URDF: " << e.what());
    return false;
  }
  
  // ROS 的硬件接口层请求对每一个具体关节（电机）的读写权限。
  numJoints_ = jointNames_.size();
  if (pinocchioModel_.nq != numJoints_) {
      ROS_ERROR_STREAM("Pinocchio model nq (" << pinocchioModel_.nq << ") does not match numJoints (" << numJoints_ << ")!");
      return false;
  }
  
  //查找yaml文件的ee_frame在pinocchio模型中的id
  if (pinocchioModel_.existFrame(eeFrame)) {
    eeFrameId_ = pinocchioModel_.getFrameId(eeFrame);
  } else {
    ROS_WARN_STREAM("EE frame '" << eeFrame << "' not found in REDUCED model, using last frame");
    eeFrameId_ = pinocchioModel_.nframes - 1;
  }

  numJoints_ = jointNames_.size();
  for (const auto& name : jointNames_) {
    try {
      jointHandles_.push_back(hw->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("Failed to get joint handle: " << name);
      return false;
    }
  }

  //初始化状态变量，位置和速度
  currentQ_.resize(numJoints_);
  currentQdot_.resize(numJoints_);
  initialQ_.resize(numJoints_);

  eePosePub_ = nh.advertise<geometry_msgs::PoseStamped>("/follower_actual_pose", 1);                //发布末端位姿
  eeTwistPub_ = nh.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);             //发布末端速度
  eeTargetPub_ = nh.advertise<geometry_msgs::PointStamped>("ee_pose_target", 1);      //发布末端目标位置
  eeDesPosePub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/follower_des_ee", 1); // [NEW] Desired EE Trajectory

  cmdPoseSub_ = nh.subscribe("command_pose", 1, &ArmController_Follower::cmdPoseCallback, this);      //接收目标位姿
  cmdTwistSub_ = nh.subscribe("command_twist", 1, &ArmController_Follower::cmdTwistCallback, this);   //接收目标速度
  cmdAccelSub_ = nh.subscribe("command_accel", 1, &ArmController_Follower::cmdAccelCallback, this);   //接收目标加速度
  cmdWrenchSub_ = nh.subscribe("command_wrench", 1, &ArmController_Follower::cmdWrenchCallback, this);//接收输出力矩

  //初始化
  targetPos_ = basePos_;      
  targetRot_.setIdentity();
  targetVel_.setZero();
  targetAngVel_.setZero();
  targetAcc_.setZero();
  targetAngAcc_.setZero();
  targetWrench_.setZero();
  hasCommand_ = false;

  calibration_rot_.setIdentity();

  sim_state_received_ = false;
  sub_object_state_ = nh.subscribe("/shared_object/state", 1, &ArmController_Follower::objectStateCallback, this);
  ROS_INFO("Follower Subscribed to /shared_object/state for GROUND TRUTH CoM state.");

  ROS_INFO("ArmController_Follower initialized successfully");
  return true;
}

void ArmController_Follower::starting(const ros::Time& time) {
  startTime_ = time;
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
    initialQ_(i) = currentQ_(i); 
  }
  
  // [DATA LOGGING] Initialize Log File (Text Format)
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream log_ss;
  log_ss << "/root/legged_ws/logs/follower_debug_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".log";
  debug_log_file_.open(log_ss.str());
  // No CSV Header needed for block text format
}

void ArmController_Follower::update(const ros::Time& time, const ros::Duration& period) {
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }

  double t = (time - startTime_).toSec();
  static double t_prev = 0.0;
  double dt = t - t_prev;
  if (dt < 1e-6) dt = 1e-6; 
  t_prev = t;

  static Robot_Follower follower_algo_;
  static bool follower_initialized = false;
  
  //Pinocchio 动力学库进行正运动学解算
  pinocchio::forwardKinematics(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::updateFramePlacements(pinocchioModel_, *pinocchioData_);
  //算出来机械臂末端现在在世界坐标系的哪个位置 (x_cur) 和朝向 (R_cur)
  Eigen::Vector3d x_cur = pinocchioData_->oMf[eeFrameId_].translation();
  Eigen::Matrix3d R_cur = pinocchioData_->oMf[eeFrameId_].rotation();

  static bool debug_frame_printed = false;
  if (!debug_frame_printed) {
      ROS_INFO_STREAM("Follower EE Pos (Using Pinocchio): " << x_cur.transpose());
      // Check arm2_base_link placement
      if (pinocchioModel_.existFrame("arm2_base_link")) {
          auto id = pinocchioModel_.getFrameId("arm2_base_link");
          ROS_INFO_STREAM("Follower Base Link Placement: \n" << pinocchioData_->oMf[id]);
      } else {
          // Fallback to checking frame 1 or 2
           ROS_WARN("Frame arm2_base_link not found in Follower model! Dumping Frame 1:");
           ROS_INFO_STREAM("Frame 1: " << pinocchioModel_.frames[1].name << " -> " << pinocchioData_->oMf[1]);
      }
      debug_frame_printed = true;
  }

  if (!follower_initialized) {
      if (!sim_state_received_) {
        ROS_WARN_THROTTLE(1.0, "Follower waiting for /shared_object/state to initialize...");
        return; 
      }       

      // Initialize algorithm with physical offset
      follower_algo_.initial(anchor_offset_); 
      follower_initialized = true;

      // Calculate initial rotation offset (Grasp Matrix Rotation Part)
      // We want R_ee = R_object * R_calib
      // So R_calib = R_object.T * R_ee_current
      calibration_rot_ = true_rot_.transpose() * R_cur;
      ROS_INFO_STREAM("Follower Calibration Rotation (Object->EE Offset):\n" << calibration_rot_);

      if (!hasCommand_) {
          targetPos_ = true_pos_; // Initialize target to CURRENT OBJECT POSITION
          targetRot_ = true_rot_;
          ROS_INFO_STREAM("Follower Initialized targetPos_ (CoM) to: " << true_pos_.transpose());
      }
  }

  State state;
  
  //计算雅可比矩阵
  Eigen::MatrixXd J(6, numJoints_);
  J.setZero();
  pinocchio::computeFrameJacobian(pinocchioModel_, *pinocchioData_, currentQ_, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  
      state.x = true_pos_;
      state.R = true_rot_;
      state.v = true_vel_;
      state.w = true_omega_;
      state.dq.resize(6);
      state.dq << state.v, state.w;

  Eigen::Vector3d a_des_ee = targetAcc_;
  Eigen::Vector3d al_des = targetAngAcc_;

  // Transform Target Acceleration to CoM
  Eigen::Vector3d r_world = R_cur * anchor_offset_; // Recalculate for current step
  Eigen::Vector3d a_des_com = a_des_ee + al_des.cross(r_world) + targetAngVel_.cross(targetAngVel_.cross(r_world));
  
  // Update order: robotupdate FIRST, then DREM
  follower_algo_.robotupdate(targetAngVel_, a_des_com, al_des, state, dt, t);
  follower_algo_.DREM(state, dt, t);

  // Output Torque is already at JOINTS level (or EE level depending on algo impl, assuming EE Wrench here)
  // [CORRECTION] Leader code implies .tau is 6D Wrench. 
  // Since we initialized with offset, this Wrench should be at EE.
  Eigen::VectorXd wrench_command = follower_algo_.tau; 

  // [DEBUG] Save RAW wrench for logging before clamping
  Eigen::VectorXd raw_wrench = wrench_command;

  static long long total_drem_steps = 0;
  static long long valid_drem_steps = 0;
  total_drem_steps++;
  bool valid = true;

  // [SAFETY CLAMP FOR IMMUTABLE ALGOTITHM]
  // Modified: Saturate instead of Zeroing out
  double safe_limit = 200.0;
  if (std::isnan(wrench_command.norm())) {
      valid = false;
      ROS_WARN_THROTTLE(0.5, "DREM Wrench NaN detected. Clamping to ZERO.");
      wrench_command.setZero();
  } else if (wrench_command.norm() > safe_limit) { 
      valid = false;
      ROS_WARN_THROTTLE(0.5, "DREM Wrench EXPLOSION detected (norm=%f). Saturated to %f.", wrench_command.norm(), safe_limit);
      wrench_command = wrench_command.normalized() * safe_limit; // Preserve direction, clamp magnitude
  }
  
  if(valid) valid_drem_steps++;
  double acceptance_rate = (total_drem_steps > 0) ? (100.0 * valid_drem_steps / total_drem_steps) : 0.0;

  Eigen::VectorXd joint_tau = J.transpose() * wrench_command;

  // Apply calibration offset to target rotation to get Desired EE Rotation
  Eigen::Matrix3d R_des = targetRot_ * calibration_rot_;
  Eigen::Vector3d x_des_com = targetPos_;
  Eigen::Vector3d xdot_des = targetVel_;
  Eigen::Vector3d omega_des = targetAngVel_;

  // [Fix Feedback Loop] Convert CoM Target -> EE Target
  Eigen::Vector3d r_des_world = R_des * anchor_offset_;
  Eigen::Vector3d x_des_ee = x_des_com - r_des_world;

  // [NEW] Publish Desired EE Position
  {
      geometry_msgs::Vector3Stamped des_ee_msg;
      des_ee_msg.header.stamp = time;
      des_ee_msg.header.frame_id = "world";
      des_ee_msg.vector.x = x_des_ee(0);
      des_ee_msg.vector.y = x_des_ee(1);
      des_ee_msg.vector.z = x_des_ee(2);
      eeDesPosePub_.publish(des_ee_msg);
  }

  // 复用已计算的 J 和 x_cur, R_cur
  double lambda = 0.01;
  Eigen::MatrixXd JJT = J * J.transpose();
  Eigen::MatrixXd JJT_damped = JJT + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd J_pinv = J.transpose() * JJT_damped.inverse();

  Eigen::Vector3d e_pos = x_des_ee - x_cur;
  Eigen::Matrix3d R_err = R_des * R_cur.transpose();
  Eigen::Vector3d e_orient;
  e_orient << 0.5 * (R_err(2,1) - R_err(1,2)),
              0.5 * (R_err(0,2) - R_err(2,0)),
              0.5 * (R_err(1,0) - R_err(0,1));

  Eigen::Matrix<double, 6, 1> e_task;
  e_task.head<3>() = e_pos;
  e_task.tail<3>() = e_orient;

  Eigen::Matrix<double, 6, 1> xdot_task_des;
  xdot_task_des.head<3>() = xdot_des;
  xdot_task_des.tail<3>() = omega_des;

  double K_pos = 10.0;
  double K_orient = 5.0;
  Eigen::Matrix<double, 6, 1> K_gains;
  K_gains << K_pos, K_pos, K_pos, K_orient, K_orient, K_orient;

  Eigen::Matrix<double, 6, 1> xdot_cmd = xdot_task_des + K_gains.asDiagonal() * e_task;

  Eigen::VectorXd qdot_ref = J_pinv * xdot_cmd;

  static Eigen::VectorXd qdot_ref_prev = Eigen::VectorXd::Zero(numJoints_);
  Eigen::VectorXd qddot_ref = Eigen::VectorXd::Zero(numJoints_);
  if (dt > 1e-6) {
    qddot_ref = (qdot_ref - qdot_ref_prev) / dt;
  }
  
  // Debug High Acceleration
  /* 
  // [DISABLED] Acceleration Feedforward causing instability due to numerical noise
  if (dt > 1e-6) {
    qddot_ref = (qdot_ref - qdot_ref_prev) / dt;
  }
  
  if (qddot_ref.norm() > 100.0) {
      ROS_WARN_THROTTLE(1.0, "High Accel Detected: qddot_ref norm = %f", qddot_ref.norm());
      qddot_ref.setZero(); // Safety Clamp
  }
  */
  qddot_ref.setZero(); // Force disable acceleration feedforward
  
  qdot_ref_prev = qdot_ref;

  pinocchio::crba(pinocchioModel_, *pinocchioData_, currentQ_);
  pinocchioData_->M.triangularView<Eigen::StrictlyLower>() = pinocchioData_->M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::computeCoriolisMatrix(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::computeGeneralizedGravity(pinocchioModel_, *pinocchioData_, currentQ_);

  // [MODIFIED] Using Gravity-Free Model (since Gazebo gravity is disabled)
  // Removed pinocchioData_->g from tau_model
  Eigen::VectorXd tau_model = pinocchioData_->M * qddot_ref + pinocchioData_->C * qdot_ref; 

  // tau_final = joint_tau (外部/DREM) + tau_model (自身动力学) + kd * (dq_ref - dq) (跟踪误差)
  Eigen::VectorXd pd_term = kd_ * (qdot_ref - currentQdot_);
  Eigen::VectorXd tau_final = tau_model + pd_term + joint_tau;

  // 保留变量定义供兼容
  Eigen::VectorXd gravity_comp = pinocchioData_->g;

  // Saturation
  double torque_limit = 30000000.0;
  for (size_t i = 0; i < numJoints_; ++i) {
      if (tau_final(i) > torque_limit) tau_final(i) = torque_limit;
      if (tau_final(i) < -torque_limit) tau_final(i) = -torque_limit;
  }

  // [DATA LOGGING] 规范化Log打印 (10Hz)
  static double last_log_time = 0;
  if (t - last_log_time > 0.1) { 
      last_log_time = t;
      
      std::stringstream ss;
      ss << "\n";
      ss << "========================= ROBOT FOLLOWER STATE =======================\n";
      char line_buf[256];
      snprintf(line_buf, sizeof(line_buf), " %-20s : %10.4f s\n", "Simulation Time", t);
      ss << line_buf;
      
      // 关节角度
      ss << " " << std::left << std::setw(20) << "Joint Angles (rad)" << " : [";
      for(size_t i=0; i<numJoints_; ++i) {
          snprintf(line_buf, sizeof(line_buf), "%7.3f%s", currentQ_(i), (i<numJoints_-1)?", ":"]\n");
          ss << line_buf;
      }

      // 关节力矩
      ss << " " << std::left << std::setw(20) << "Joint Torques (Nm)" << " : [";
      for(size_t i=0; i<numJoints_; ++i) {
          snprintf(line_buf, sizeof(line_buf), "%7.2f%s", tau_final(i), (i<numJoints_-1)?", ":"]\n");
          ss << line_buf;
      }

      snprintf(line_buf, sizeof(line_buf), " %-20s : %10.4f\n", "DREM Force Norm", raw_wrench.norm());
      ss << line_buf;

      // Log RAW DREM Wrench Vector (Before Clamping)
      ss << " " << std::left << std::setw(20) << "Raw Wrench" << " : [";
      if (raw_wrench.size() == 6) {
           for(long i=0; i<raw_wrench.size(); ++i) {
               snprintf(line_buf, sizeof(line_buf), "%7.1f%s", raw_wrench(i), (i<raw_wrench.size()-1)?", ":"]\n");
               ss << line_buf;
           }
      } else {
           ss << "Size Error]\n";
      }

      // [NEW] Log Estimated Mass
      Eigen::VectorXd estimates = follower_algo_.get_estimated_params();
      double est_mass = (estimates.size() > 0) ? estimates(0) : 0.0;
      snprintf(line_buf, sizeof(line_buf), " %-20s : %10.4f kg\n", "Est. Object Mass", est_mass);
      ss << line_buf;

      // Log detailed parameters
      if (estimates.size() == 10) {
          ss << " " << std::left << std::setw(20) << "Est. COM Moment" << " : [";
          for(int i=1; i<=3; ++i) {
              snprintf(line_buf, sizeof(line_buf), "%7.4f%s", estimates(i), (i<3)?", ":"]\n");
              ss << line_buf;
          }
          ss << " " << std::left << std::setw(20) << "Est. Inertia Iso" << " : [";
          for(int i=4; i<10; ++i) {
              snprintf(line_buf, sizeof(line_buf), "%7.4f%s", estimates(i), (i<9)?", ":"]\n");
              ss << line_buf;
          }
      }

      snprintf(line_buf, sizeof(line_buf), " %-20s : %10.2f %%\n", "DREM Accept Rate", acceptance_rate);
      ss << line_buf;
      ss << "======================================================================\n";

      std::string log_str = ss.str();
      
      // Print to Terminal
      std::cout << log_str;
      
      // Write to File
      if (debug_log_file_.is_open()) {
          debug_log_file_ << log_str;
      }
  }

  //发送力矩到各个关节
  for (size_t i = 0; i < numJoints_; ++i) {
    jointHandles_[i].setCommand(tau_final(i));
  }
  
  //绘图用，发布当前末端位置，速度，目标位置
  geometry_msgs::PointStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = "world";
  msg.point.x = x_cur(0);
  msg.point.y = x_cur(1);
  msg.point.z = x_cur(2);
  eeTargetPub_.publish(msg);
  //转换并发布末端位姿
  const auto& eePose = pinocchioData_->oMf[eeFrameId_];
  const auto& translation = eePose.translation();
  const auto& rotation = Eigen::Quaterniond(eePose.rotation());

  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.stamp = time;
  poseMsg.header.frame_id = "base_link";
  poseMsg.pose.position.x = translation.x();
  poseMsg.pose.position.y = translation.y();
  poseMsg.pose.position.z = translation.z();
  poseMsg.pose.orientation.x = rotation.x();
  poseMsg.pose.orientation.y = rotation.y();
  poseMsg.pose.orientation.z = rotation.z();
  poseMsg.pose.orientation.w = rotation.w();
  eePosePub_.publish(poseMsg);

  //发布末端速度
  Eigen::VectorXd v_cur = J * currentQdot_; // 6D Twist
  geometry_msgs::TwistStamped twistMsg;
  twistMsg.header.stamp = time;
  twistMsg.header.frame_id = "base_link";
  twistMsg.twist.linear.x = v_cur(0);
  twistMsg.twist.linear.y = v_cur(1);
  twistMsg.twist.linear.z = v_cur(2);
  twistMsg.twist.angular.x = v_cur(3);
  twistMsg.twist.angular.y = v_cur(4);
  twistMsg.twist.angular.z = v_cur(5);
  eeTwistPub_.publish(twistMsg);
}

void ArmController_Follower::stopping(const ros::Time& time) {
}

//接收my_controllers的follower发回来的位姿的回调函数
void ArmController_Follower::cmdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  targetPos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  targetRot_ = q.toRotationMatrix();
  hasCommand_ = true;
}
//接收my_controllers的follower发回来的速度的回调函数
void ArmController_Follower::cmdTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  targetVel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  targetAngVel_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}
void ArmController_Follower::cmdAccelCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
  targetAcc_ << msg->accel.linear.x, msg->accel.linear.y, msg->accel.linear.z;
  targetAngAcc_ << msg->accel.angular.x, msg->accel.angular.y, msg->accel.angular.z;
}

//接收my_controllers的follower发回来的力矩的回调函数
void ArmController_Follower::cmdWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  targetWrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                   msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

void ArmController_Follower::objectStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Directly extract state from Odometry message
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  const auto& v = msg->twist.twist.linear;
  const auto& w = msg->twist.twist.angular;

  true_pos_ << p.x, p.y, p.z;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  true_rot_ = quat.toRotationMatrix();
  true_vel_ << v.x, v.y, v.z;
  true_omega_ << w.x, w.y, w.z;

  sim_state_received_ = true;
}

} // namespace arm_control
PLUGINLIB_EXPORT_CLASS(arm_control::ArmController_Follower, controller_interface::ControllerBase)
