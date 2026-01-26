#include "arm_control/ArmController.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <algorithm>

namespace arm_control {

bool ArmController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  nh_ = nh;

  // 1. Load Parameters
  std::string urdfFile, eeFrame;
  nh.param<std::string>("urdf_file", urdfFile, "");
  nh.param<std::string>("ee_frame", eeFrame, "tool0");
  nh.param<double>("kp", kp_, 100.0);
  nh.param<double>("kd", kd_, 10.0);
  
  // Sine Wave Params
  nh.param<double>("amplitude_x", amplitudeX_, 0.0);
  nh.param<double>("amplitude_y", amplitudeY_, 0.0);
  nh.param<double>("amplitude_z", amplitudeZ_, 0.0);
  nh.param<double>("frequency", frequency_, 0.2);
  nh.param<double>("phase_x", phaseX_, 0.0);
  nh.param<double>("phase_y", phaseY_, 0.0);
  nh.param<double>("phase_z", phaseZ_, M_PI / 2.0);
  
  double bx, by, bz;
  nh.param<double>("base_pos_x", bx, 0.0);
  nh.param<double>("base_pos_y", by, 0.0);
  nh.param<double>("base_pos_z", bz, 0.8);
  basePos_ << bx, by, bz;

  // 2. Load URDF & Initialize Pinocchio
  if (urdfFile.empty()) {
    std::string urdfString;
    std::string robotDescriptionParam;
    if (nh.searchParam("robot_description", robotDescriptionParam)) {
      nh.getParam(robotDescriptionParam, urdfString);
    } else if (nh.getParam("/robot_description", urdfString)) {
       // Fallback to global
    }

    if (!urdfString.empty()) {
       // 写入临时文件
       std::string ns = nh.getNamespace();
       std::replace(ns.begin(), ns.end(), '/', '_');
       urdfFile = "/tmp/temp_arm_control" + ns + ".urdf";
       std::ofstream out(urdfFile);
       out << urdfString;
       out.close();
    } else {
       ROS_ERROR("Please provide urdf_file parameter or robot_description");
       return false;
    }
  }
  
  try {
    pinocchio::urdf::buildModel(urdfFile, pinocchioModel_);
    pinocchioData_ = std::make_unique<pinocchio::Data>(pinocchioModel_);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to load URDF: " << e.what());
    return false;
  }

  if (pinocchioModel_.existFrame(eeFrame)) {
    eeFrameId_ = pinocchioModel_.getFrameId(eeFrame);
  } else {
    eeFrameId_ = pinocchioModel_.nframes - 1;
  }

  // 3. Get Joint Handles
  jointNames_ = {"robot2_joint_a1", "robot2_joint_a2", "robot2_joint_a3",
                 "robot2_joint_a4", "robot2_joint_a5", "robot2_joint_a6",
                 "robot2_joint_a7"}; 
  if (nh.hasParam("joint_names")) nh.getParam("joint_names", jointNames_);

  numJoints_ = jointNames_.size();
  for (const auto& name : jointNames_) {
    try {
      jointHandles_.push_back(hw->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("Failed to get joint handle: " << name);
      return false;
    }
  }

  currentQ_.resize(numJoints_);
  currentQdot_.resize(numJoints_);

  // 4. Publishers
  eePosePub_ = nh.advertise<geometry_msgs::PoseStamped>("ee_pose", 1);
  eeTargetPub_ = nh.advertise<geometry_msgs::PointStamped>("ee_pose_target", 1);

  return true;
}

void ArmController::starting(const ros::Time& time) {
  startTime_ = time;
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }
}

void ArmController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. Update State
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }

  double t = (time - startTime_).toSec();

  // 2. Generate Reference (Sine Wave)
  Eigen::Vector3d x_des = basePos_;
  x_des.x() += amplitudeX_ * sin(2 * M_PI * frequency_ * t + phaseX_);
  x_des.y() += amplitudeY_ * sin(2 * M_PI * frequency_ * t + phaseY_);
  x_des.z() += amplitudeZ_ * sin(2 * M_PI * frequency_ * t + phaseZ_);

  Eigen::Vector3d xdot_des = Eigen::Vector3d::Zero();
  xdot_des.x() = amplitudeX_ * 2 * M_PI * frequency_ * cos(2 * M_PI * frequency_ * t + phaseX_);
  xdot_des.y() = amplitudeY_ * 2 * M_PI * frequency_ * cos(2 * M_PI * frequency_ * t + phaseY_);
  xdot_des.z() = amplitudeZ_ * 2 * M_PI * frequency_ * cos(2 * M_PI * frequency_ * t + phaseZ_);

  // 3. Kinematics & Jacobian
  pinocchio::forwardKinematics(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::updateFramePlacements(pinocchioModel_, *pinocchioData_);
  
  Eigen::Vector3d x_cur = pinocchioData_->oMf[eeFrameId_].translation();
  Eigen::Matrix3d R_cur = pinocchioData_->oMf[eeFrameId_].rotation();
  
  Eigen::MatrixXd J(6, numJoints_);
  J.setZero();
  pinocchio::computeFrameJacobian(pinocchioModel_, *pinocchioData_, currentQ_, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 4. IK (CLIK)
  Eigen::Matrix3d R_des;
  R_des << 1,  0,  0,
           0, -1,  0,
           0,  0, -1;
  Eigen::Vector3d omega_des = Eigen::Vector3d::Zero();

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
  static double t_prev = 0.0;
  double dt = t - t_prev;
  Eigen::VectorXd qddot_ref = Eigen::VectorXd::Zero(numJoints_);
  if (dt > 1e-6) {
    qddot_ref = (qdot_ref - qdot_ref_prev) / dt;
  }
  qdot_ref_prev = qdot_ref;
  t_prev = t;
  
  // 5. ID (Computed Torque)
  pinocchio::crba(pinocchioModel_, *pinocchioData_, currentQ_);
  pinocchioData_->M.triangularView<Eigen::StrictlyLower>() = pinocchioData_->M.transpose().triangularView<Eigen::StrictlyLower>();
  
  pinocchio::computeCoriolisMatrix(pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_);
  pinocchio::computeGeneralizedGravity(pinocchioModel_, *pinocchioData_, currentQ_);

  Eigen::VectorXd tau_model = pinocchioData_->M * qddot_ref + pinocchioData_->C * qdot_ref + pinocchioData_->g;
  Eigen::VectorXd tau = tau_model + kd_ * (qdot_ref - currentQdot_);
  
  // 6. Send Command
  for (size_t i = 0; i < numJoints_; ++i) {
    jointHandles_[i].setCommand(tau(i));
  }
  
  // 7. Publish Visualization
  geometry_msgs::PointStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = "world";
  msg.point.x = x_des(0);
  msg.point.y = x_des(1);
  msg.point.z = x_des(2);
  eeTargetPub_.publish(msg);

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
}

void ArmController::stopping(const ros::Time& time) {
}

} // namespace arm_control
PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase)
