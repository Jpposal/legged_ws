#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace arm_control {

class ArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  ArmController() = default;
  ~ArmController() override = default;

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

  // Gains & Params
  double kp_, kd_;
  double amplitudeX_, amplitudeY_, amplitudeZ_, frequency_;
  double phaseX_, phaseY_, phaseZ_;
  Eigen::Vector3d basePos_;

  // Time
  ros::Time startTime_;

  // Publishers
  ros::Publisher eePosePub_;
  ros::Publisher eeTargetPub_;
};

} // namespace arm_control
