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
#include <fstream>

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

  eePosePub_ = nh.advertise<geometry_msgs::PoseStamped>("ee_pose", 1);                //发布末端位姿
  eeTwistPub_ = nh.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);             //发布末端速度
  eeTargetPub_ = nh.advertise<geometry_msgs::PointStamped>("ee_pose_target", 1);      //发布末端目标位置

  cmdPoseSub_ = nh.subscribe("command_pose", 1, &ArmController_Follower::cmdPoseCallback, this);      //接收目标位姿
  cmdTwistSub_ = nh.subscribe("command_twist", 1, &ArmController_Follower::cmdTwistCallback, this);   //接收目标速度
  cmdWrenchSub_ = nh.subscribe("command_wrench", 1, &ArmController_Follower::cmdWrenchCallback, this);//接收输出力矩

  //初始化
  targetPos_ = basePos_;      
  targetRot_.setIdentity();
  targetVel_.setZero();
  targetAngVel_.setZero();
  targetWrench_.setZero();
  hasCommand_ = false;

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

  if (!follower_initialized) {
      // Use current pose for initialization to avoid jump
      follower_algo_.initial(Eigen::Vector3d::Zero(), x_cur, R_cur); 
      follower_initialized = true;
      ROS_INFO_STREAM("Follower Initialized at: " << x_cur.transpose());
  }

  State state;
  
  //计算雅可比矩阵
  Eigen::MatrixXd J(6, numJoints_);
  J.setZero();
  pinocchio::computeFrameJacobian(pinocchioModel_, *pinocchioData_, currentQ_, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  Eigen::VectorXd v_cur_spatial = J * currentQdot_;       //计算末端空间速度
  //更新状态
  state.x = x_cur;
  state.R = R_cur;
  state.v = v_cur_spatial.head<3>();
  state.w = v_cur_spatial.tail<3>();
  
  state.dq.resize(6);
  
  // if (!hasCommand_) {
  //     ROS_WARN_THROTTLE(2.0, "Follower Waiting for command... Holding position.");
  // }
  
  // Update order: robotupdate FIRST, then DREM (consistent with Leader logic)
  follower_algo_.robotupdate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), state, dt, t);
  follower_algo_.DREM(state, dt, t);

  Eigen::VectorXd wrench_command = follower_algo_.tau; 
  Eigen::VectorXd joint_tau = J.transpose() * wrench_command;

  // [Added] 1. CLIK (Inverse Kinematics) - 移植自 ArmController.cpp
  Eigen::Matrix3d R_des = targetRot_;
  Eigen::Vector3d x_des = targetPos_;
  Eigen::Vector3d xdot_des = targetVel_;
  Eigen::Vector3d omega_des = targetAngVel_;

  // 复用已计算的 J 和 x_cur, R_cur
  double lambda = 0.01;
  Eigen::MatrixXd JJT = J * J.transpose();
  Eigen::MatrixXd JJT_damped = JJT + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd J_pinv = J.transpose() * JJT_damped.inverse();

  Eigen::Vector3d e_pos = x_des - x_cur;
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
  qdot_ref_prev = qdot_ref;

  // [Added] 2. ID (Computed Torque) - 移植自 ArmController.cpp
  pinocchio::crba(pinocchioModel_, *pinocchioData_, currentQ_);
  pinocchioData_->M.triangularView<Eigen::StrictlyLower>() = pinocchioData_->M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::computeCoriolisMatrix(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::computeGeneralizedGravity(pinocchioModel_, *pinocchioData_, currentQ_);

  Eigen::VectorXd tau_model = pinocchioData_->M * qddot_ref + pinocchioData_->C * qdot_ref + pinocchioData_->g;

  // [Modified] 3. Final Torque Calculation
  // tau_final = joint_tau (外部/DREM) + tau_model (自身动力学) + kd * (dq_ref - dq) (跟踪误差)
  Eigen::VectorXd tau_final = tau_model + kd_ * (qdot_ref - currentQdot_) + joint_tau;

  // 保留变量定义供兼容
  Eigen::VectorXd gravity_comp = pinocchioData_->g;

  // Saturation
  double torque_limit = 300.0;
  for (size_t i = 0; i < numJoints_; ++i) {
      if (tau_final(i) > torque_limit) tau_final(i) = torque_limit;
      if (tau_final(i) < -torque_limit) tau_final(i) = -torque_limit;
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
//接收my_controllers的follower发回来的力矩的回调函数
void ArmController_Follower::cmdWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  targetWrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                   msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

} // namespace arm_control
PLUGINLIB_EXPORT_CLASS(arm_control::ArmController_Follower, controller_interface::ControllerBase)
