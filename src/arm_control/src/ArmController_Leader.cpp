#include "arm_control/ArmController_Leader.h"
#include <my_robot_controllers/robot_leader.h>
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
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <algorithm>

namespace arm_control {

bool ArmController_Leader::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  nh_ = nh;

  //这里是读取通过dual_arm_controllers.yaml加载的参数
  //dual_arm_controllers.yaml加载的参数是根据自己设定的
  std::string urdfFile, eeFrame;
  nh.param<std::string>("urdf_file", urdfFile, "");
  std::cerr << "urdf_file param: " << urdfFile << std::endl;
  nh.param<std::string>("ee_frame", eeFrame, "tool0");
  std::cerr << "ee_frame param: " << eeFrame << std::endl;
  nh.param<double>("kp", kp_, 100.0);
  nh.param<double>("kd", kd_, 10.0);
  nh.param<bool>("use_drem", use_drem_, true);

  // if (!use_drem_) {
  //     ROS_WARN("DREM and Task Space Algo DISABLED. Running pure Joint ID Control.");
  // }
  
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
  ROS_INFO_STREAM("Leader Anchor Offset (EE->CoM): " << anchor_offset_.transpose());

  //还是读取通过dual_arm_controllers.yaml加载的参数
  if (urdfFile.empty()) {
    std::string robotDescriptionParam;
    if (nh_.getParam("robot_description_param", robotDescriptionParam)) {
       ROS_INFO_STREAM("Using robot_description_param: " << robotDescriptionParam);
    } else {
       ROS_ERROR("Failed to find 'robot_description_param' in controller configuration. Please set it in your .yaml file.");
       return false;
    }
  //读取单机械臂的urdf
    std::string urdfString;
    if (nh_.getParam(robotDescriptionParam, urdfString)) {
       ROS_INFO_STREAM("Found URDF string at parameter: " << robotDescriptionParam);
    } else {
       ROS_ERROR_STREAM("Failed to find URDF string at parameter: " << robotDescriptionParam);
       return false;
    }
    //缓存单机械臂的urdf
    if (!urdfString.empty()) {
       std::string ns = nh_.getNamespace();
       std::replace(ns.begin(), ns.end(), '/', '_');
       urdfFile = "/tmp/temp_arm_control_leader" + ns + ".urdf";
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
    
    if (pinocchioModel_.nq != jointNames_.size()) {
       ROS_WARN_STREAM("Built Pinocchio model has nq=" << pinocchioModel_.nq << " but we have " << jointNames_.size() << " joints in param. Ensure your URDF is single-arm only.");
    }

    std::cerr << "Pinocchio Model built. Printing frames (nframes=" << pinocchioModel_.nframes << "):" << std::endl;
    for (int i = 0; i < pinocchioModel_.nframes; ++i) {
        const auto& f = pinocchioModel_.frames[i];
        std::cerr << "Frame " << i << ": Name='" << f.name << "', Type=" << f.type << std::endl;
        if (f.name == eeFrame) {
             std::cerr << "MATCH FOUND for '" << eeFrame << "' at index " << i << std::endl;
        }
    }

    pinocchioData_ = std::make_unique<pinocchio::Data>(pinocchioModel_);
    
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to load URDF: " << e.what());
    return false;
  }

  //查找yaml文件的ee_frame在pinocchio模型中的id
  std::cerr << "Checking eeFrame: " << eeFrame << std::endl;
  bool frame_exists = false;
  for(const auto& f : pinocchioModel_.frames) {
      if(f.name == eeFrame) frame_exists = true;
  }
  
  if (frame_exists) {
    eeFrameId_ = pinocchioModel_.getFrameId(eeFrame);
  } else {
    ROS_WARN_STREAM("EE frame '" << eeFrame << "' not found in REDUCED model, using last frame");
    eeFrameId_ = pinocchioModel_.nframes - 1;
  }
  // ROS 的硬件接口层请求对每一个具体关节（电机）的读写权限。
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
  //检查pinocchio模型的自由度和关节数是否匹配
  if (pinocchioModel_.nq != numJoints_) {
    ROS_ERROR_STREAM("Pinocchio model nq (" << pinocchioModel_.nq << ") does not match numJoints (" << numJoints_ << ")!");
    return false;
  }
  if (pinocchioModel_.nv != numJoints_) {
    ROS_ERROR_STREAM("Pinocchio model nv (" << pinocchioModel_.nv << ") does not match numJoints (" << numJoints_ << ")!");
    return false;
  }

  eePosePub_ = nh.advertise<geometry_msgs::PoseStamped>("/leader_actual_pose", 1);                //发布末端位姿
  eeTwistPub_ = nh.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);             //发布末端速度
  eeTargetPub_ = nh.advertise<geometry_msgs::PointStamped>("/leader_target_pose", 1);      //发布末端目标位置
  dremParamsPub_ = nh.advertise<std_msgs::Float64MultiArray>("/leader_drem_params", 1); // 发布DREM参数

  cmdPoseSub_ = nh.subscribe("command_pose", 1, &ArmController_Leader::cmdPoseCallback, this);      //接收目标位姿
  cmdTwistSub_ = nh.subscribe("command_twist", 1, &ArmController_Leader::cmdTwistCallback, this);   //接收目标速度
  cmdWrenchSub_ = nh.subscribe("command_wrench", 1, &ArmController_Leader::cmdWrenchCallback, this);//接收输出力矩

  //初始化
  targetPos_ = basePos_;      
  targetRot_.setIdentity();
  targetVel_.setZero();
  targetAngVel_.setZero();
  targetWrench_.setZero();
  hasCommand_ = false;

  // Initialize Gazebo Truth (Now via Standardized Odom Topic)
  sim_state_received_ = false;
  sub_object_state_ = nh.subscribe("/shared_object/state", 1, &ArmController_Leader::objectStateCallback, this);
  ROS_INFO("Subscribed to /shared_object/state for GROUND TRUTH CoM state.");

  ROS_INFO("ArmController_Leader initialized successfully");
  return true;
}

void ArmController_Leader::starting(const ros::Time& time) {
  startTime_ = time;
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
    initialQ_(i) = currentQ_(i); 
  }
}

void ArmController_Leader::update(const ros::Time& time, const ros::Duration& period) {
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }

  double t = (time - startTime_).toSec();
  static double t_prev = 0.0;
  double dt = t - t_prev;
  if (dt < 1e-6) dt = 1e-6;
  t_prev = t;

  static Robot_Leader leader_algo_;
  static bool leader_initialized = false;
  static Eigen::Vector3d traj_center = Eigen::Vector3d::Zero();
  
  //Pinocchio 动力学库进行正运动学解算
  pinocchio::forwardKinematics(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::updateFramePlacements(pinocchioModel_, *pinocchioData_);
  //算出来机械臂末端现在在世界坐标系的哪个位置 (x_cur) 和朝向 (R_cur)
  Eigen::Vector3d x_cur_ee = pinocchioData_->oMf[eeFrameId_].translation();
  Eigen::Matrix3d R_cur = pinocchioData_->oMf[eeFrameId_].rotation();
  Eigen::Vector3d x_cur_com = x_cur_ee - R_cur * anchor_offset_;

  // [MODIFIED] Initialization Logic: Wait for Ground Truth
  if (!leader_initialized) {
      if (!sim_state_received_) {
        // If we haven't received the object state yet, we cannot initialize the controller safely.
        // We skip the control loop this cycle (maintaining zero/gravity torque or holding).
        ROS_WARN_THROTTLE(1.0, "Waiting for /shared_object/state to initialize controller...");
        return; 
      }

      // Initialize algorithm with physical offset
      leader_algo_.initial(anchor_offset_); 
      
      // Use the received GROUND TRUTH for initialization
      traj_center = true_pos_; 

      if (!hasCommand_) {
          targetPos_ = true_pos_; // Initialize target to CURRENT OBJECT POSITION
          targetRot_ = true_rot_;
          ROS_INFO_STREAM("Initialized targetPos_ (CoM) from GT to: " << true_pos_.transpose());
      }

      leader_initialized = true;
  }

  Target target;
  State state;
  
  //计算雅可比矩阵
  Eigen::MatrixXd J(6, numJoints_);
  J.setZero();
  pinocchio::computeFrameJacobian(pinocchioModel_, *pinocchioData_, currentQ_, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  
  // Directly use Received Object State (Sim-to-Real Architecture)
  if (sim_state_received_) {
      state.x = true_pos_;
      state.R = true_rot_;
      state.v = true_vel_;
      state.w = true_omega_;
      state.dq.resize(6);
      state.dq << state.v, state.w;
  } else {
       // Fallback for safety until first message
       state.x = x_cur_com; 
       state.R = R_cur;
       state.v.setZero();
       state.w.setZero();
       state.dq.resize(6);
       state.dq.setZero();
  }


  if (!hasCommand_) {
    ROS_WARN_THROTTLE(2.0, "Waiting for command... Robot holding initial position.");
  }
  
  // [Target Assignment]
  // targetPos_ is the DESIRED CoM Position (from Object Target Publisher)
  // No transformation needed as the publisher sends CoM targets directly.
  
  Eigen::Matrix3d R_d = targetRot_;
  
  target.x_d = targetPos_;
  target.v_d = targetVel_;
  target.R_d = targetRot_;
  target.w_d = targetAngVel_;
  target.a_d = Eigen::Vector3d::Zero(); // Simplified: Ignoring w_dot x r for now or assuming small
  target.al_d = Eigen::Vector3d::Zero();

  Eigen::VectorXd wrench_command(6);
  Eigen::VectorXd joint_tau(numJoints_);

  leader_algo_.robotupdate(target, state, dt);
  leader_algo_.DREM(target, state, dt, t);
  wrench_command = leader_algo_.tau;

  static long long total_drem_steps = 0;
  static long long valid_drem_steps = 0;
  total_drem_steps++;
  bool valid = true;

  // [SAFETY CLAMP FOR IMMUTABLE ALGORITHM]
  if (wrench_command.norm() > 200.0 || std::isnan(wrench_command.norm())) {
      valid = false;
      ROS_WARN_THROTTLE(0.5, "LEADER DREM Wrench EXPLOSION detected (norm=%f). Clamping to ZERO to protect sim.", wrench_command.norm());
      wrench_command.setZero();
  }
  
  if(valid) valid_drem_steps++;
  double acceptance_rate = (total_drem_steps > 0) ? (100.0 * valid_drem_steps / total_drem_steps) : 0.0;

  joint_tau = J.transpose() * wrench_command;

  Eigen::Matrix3d R_des = target.R_d;
  Eigen::Vector3d x_des_com = target.x_d; // This is CoM Desired
  Eigen::Vector3d xdot_des = target.v_d;
  Eigen::Vector3d omega_des = target.w_d;

  // [Fix Feedback Loop] Convert CoM Target -> EE Target
  // x_ee_des = x_com_des - R_des * offset
  Eigen::Vector3d r_des_world = R_des * anchor_offset_;
  Eigen::Vector3d x_des_ee = x_des_com - r_des_world;

  double lambda = 0.01;
  Eigen::MatrixXd JJT = J * J.transpose();
  Eigen::MatrixXd JJT_damped = JJT + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd J_pinv = J.transpose() * JJT_damped.inverse();

  // Calculate error in EE Frame
  Eigen::Vector3d e_pos = x_des_ee - x_cur_ee;
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

  double K_pos = 100.0;
  double K_orient = 50.0;
  Eigen::Matrix<double, 6, 1> K_gains;
  K_gains << K_pos, K_pos, K_pos, K_orient, K_orient, K_orient;

  Eigen::Matrix<double, 6, 1> xdot_cmd = xdot_task_des + K_gains.asDiagonal() * e_task;

  Eigen::VectorXd qdot_ref = J_pinv * xdot_cmd;

  static Eigen::VectorXd qdot_ref_prev = Eigen::VectorXd::Zero(numJoints_);
  Eigen::VectorXd qddot_ref = Eigen::VectorXd::Zero(numJoints_);
  if (dt > 1e-6) {
    qddot_ref = (qdot_ref - qdot_ref_prev) / dt;
  }
  qdot_ref_prev = qdot_ref;

  pinocchio::crba(pinocchioModel_, *pinocchioData_, currentQ_);
  pinocchioData_->M.triangularView<Eigen::StrictlyLower>() = pinocchioData_->M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::computeCoriolisMatrix(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::computeGeneralizedGravity(pinocchioModel_, *pinocchioData_, currentQ_);

  Eigen::VectorXd tau_model = pinocchioData_->M * qddot_ref + pinocchioData_->C * qdot_ref; // + pinocchioData_->g;

  Eigen::VectorXd tau_final = tau_model + kd_ * (qdot_ref - currentQdot_) + joint_tau;

  Eigen::VectorXd gravity_comp = pinocchioData_->g;

  //增加力矩限幅
  double torque_limit = 30000000.0;
  for (size_t i = 0; i < numJoints_; ++i) {
      if (tau_final(i) > torque_limit) tau_final(i) = torque_limit;
      if (tau_final(i) < -torque_limit) tau_final(i) = -torque_limit;
  }

  // 发布DREM参数，以便分析发散原因
  // std_msgs::Float64MultiArray params_msg;
  // Eigen::VectorXd hats = leader_algo_.get_hat_o();
  // for(int i=0; i<10; i++) params_msg.data.push_back(hats(i));
  // dremParamsPub_.publish(params_msg);

  // 打印debug信息：维度、算法输出力矩、雅可比转换力矩、最终下发力矩
  // 增加打印：位置误差
  Eigen::Vector3d pos_error = target.x_d - state.x;
  // [DEBUG] 增加了打印 gravity_comp(1) 以便观察重力矩大小
  ROS_INFO_THROTTLE(0.5, "ERR_Pos: %.4f %.4f %.4f | G_Tau: %.2f | Final_Tau: %.2f | Acc_Rate: %.1f%%",
      pos_error(0), pos_error(1), pos_error(2), 
      gravity_comp(1),   // 关节2的重力矩
      tau_final(1),     // 最终下发给关节2的力矩
      acceptance_rate); 
  // -------------------

  //发送力矩到各个关节
  for (size_t i = 0; i < numJoints_; ++i) {
    jointHandles_[i].setCommand(tau_final(i));
  }
  
  //绘图用，发布当前末端位置，速度，目标位置
  geometry_msgs::PointStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = "world";
  msg.point.x = target.x_d(0);
  msg.point.y = target.x_d(1);
  msg.point.z = target.x_d(2);
  eeTargetPub_.publish(msg);
  //转换并发布末端位姿
  const auto& eePose = pinocchioData_->oMf[eeFrameId_];
  const auto& translation = eePose.translation();
  const auto& rotation = Eigen::Quaterniond(eePose.rotation());

  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.stamp = time;
  poseMsg.header.frame_id = "world";
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
  twistMsg.header.frame_id = "world";
  twistMsg.twist.linear.x = v_cur(0);
  twistMsg.twist.linear.y = v_cur(1);
  twistMsg.twist.linear.z = v_cur(2);
  twistMsg.twist.angular.x = v_cur(3);
  twistMsg.twist.angular.y = v_cur(4);
  twistMsg.twist.angular.z = v_cur(5);
  eeTwistPub_.publish(twistMsg);
}

void ArmController_Leader::stopping(const ros::Time& time) {
}
//接收my_controllers的leader发回来的位姿的回调函数
void ArmController_Leader::cmdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  targetPos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  targetRot_ = q.toRotationMatrix();
  hasCommand_ = true;
}
//接收my_controllers的leader发回来的速度的回调函数
void ArmController_Leader::cmdTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  targetVel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  targetAngVel_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}
//接收my_controllers的leader发回来的力矩的回调函数
void ArmController_Leader::cmdWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  targetWrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                   msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

void ArmController_Leader::objectStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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

}
PLUGINLIB_EXPORT_CLASS(arm_control::ArmController_Leader, controller_interface::ControllerBase)
