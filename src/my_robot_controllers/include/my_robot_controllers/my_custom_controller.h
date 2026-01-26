#ifndef MY_ROBOT_CONTROLLERS__MY_CUSTOM_CONTROLLER_H
#define MY_ROBOT_CONTROLLERS__MY_CUSTOM_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <CRBT_common/MotionCaptureInterface.h>
#include <CRBT_common/SendCurrentsInterface.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_interface/multi_interface_controller.h>
#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/Geometry> 

#include "my_robot_controllers/robot_leader.h"
#include "my_robot_controllers/robot_follower.h"

#include "state.h"

using namespace Eigen;

namespace CRBT {


struct USE {
  double position_[2];      // 2D position
  double velocity_[2];      // 2D velocity
  double attitude_;         // 1D attitude (angle)
  double angular_velocity_; // 1D angular velocity
  // state
  
};
class MyCustomController : public controller_interface::MultiInterfaceController<MotionCaptureInterface, SendCurrentsInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

protected:
    MotionCaptureHandle motionCaptureHandles_;
    SendCurrentsHandle sendCurrentsHandles_;
    

private:
    Vector3d r_i;
    State state;
    Target target;
    ros::Time last_time;
    double t;
    double dt;
    Robot_Leader robot_leader;
    Robot_Follower robot_follower;
    Eigen::VectorXd tau;
    MotionCaptureInterface motionCaptureInterface_;
    SendCurrentsInterface sendCurrentsInterface_;
    //double new_currents[4];
    Eigen::Quaterniond qua;
};

} // namespace CRBT

#endif // MY_ROBOT_CONTROLLERS__MY_CUSTOM_CONTROLLER_H
