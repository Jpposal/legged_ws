#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace arm_control {

class ArmController_Follower : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  ArmController_Follower() = default;
  ~ArmController_Follower() override = default;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  // ROS handles
  ros::NodeHandle nh_;
  std::vector<hardware_interface::JointHandle> jointHandles_;
  std::vector<std::string> jointNames_;

  // Pinocchio
  pinocchio::Model pinocchioModel_;
  std::unique_ptr<pinocchio::Data> pinocchioData_;
  pinocchio::FrameIndex eeFrameId_;

  // State
  size_t numJoints_;
  Eigen::VectorXd currentQ_;
  Eigen::VectorXd currentQdot_;
  Eigen::VectorXd initialQ_; // To store initial pose for kp spring effect

  // Gains & Params
  double kp_, kd_;
  Eigen::Vector3d basePos_;
  Eigen::Vector3d anchor_offset_;

  // Time
  ros::Time startTime_;

  // Publishers
  ros::Publisher eePosePub_;
  ros::Publisher eeTwistPub_;
  ros::Publisher eeTargetPub_;

  // Subscribers
  ros::Subscriber cmdPoseSub_;
  ros::Subscriber cmdTwistSub_;
  ros::Subscriber cmdAccelSub_;
  ros::Subscriber cmdWrenchSub_;

  // Command Data
  Eigen::Vector3d targetPos_;
  Eigen::Matrix3d targetRot_;
  Eigen::Vector3d targetVel_;
  Eigen::Vector3d targetAngVel_;
  Eigen::Vector3d targetAcc_;
  Eigen::Vector3d targetAngAcc_;
  Eigen::Matrix<double, 6, 1> targetWrench_;
  bool hasCommand_;

  // Callbacks
  void cmdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void cmdTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void cmdAccelCallback(const geometry_msgs::AccelStamped::ConstPtr& msg);
  void cmdWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void objectStateCallback(const nav_msgs::Odometry::ConstPtr& msg);

  // Gazebo Truth Data
  ros::Subscriber sub_object_state_;
  bool sim_state_received_;
  Eigen::Vector3d true_pos_;
  Eigen::Vector3d true_vel_;
  Eigen::Vector3d true_omega_;
  Eigen::Matrix3d true_rot_;
  
  // Calibration
  Eigen::Matrix3d calibration_rot_;
};

} // namespace arm_control
