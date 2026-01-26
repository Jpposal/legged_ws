#include "my_robot_controllers/my_custom_controller.h"
#include "my_robot_controllers/funs.h"
#include "my_robot_controllers/convert.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <cstddef>  // 用于定义 NULL
#include <array>

namespace CRBT {

bool MyCustomController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
    ros::NodeHandle nh;
    MotionCaptureInterface* motioncapture_hw = robot_hw->get<MotionCaptureInterface>();
    SendCurrentsInterface* sendcurrents_hw = robot_hw->get<SendCurrentsInterface>();
    /*
    if (!vel_hw || !state_hw) {
        ROS_ERROR("This controller requires a hardware interface of type VelocityJointInterface and JointStateInterface.");
        return false;
    }
    */

    /*
    std::string lin_wheel_name, ang_wheel_name;
    if (!nh.getParam("lin_wheel", lin_wheel_name) || !nh.getParam("ang_wheel", ang_wheel_name)) {
        ROS_ERROR("Could not find wheel names");
        return false;
    }
    */
    
    // Hardware interface
    auto* motionCaptureInterface = robot_hw->get<MotionCaptureInterface>();
    motionCaptureHandles_ = motionCaptureInterface->getHandle("object1");
    auto* sendCurrentsInterface = robot_hw->get<SendCurrentsInterface>();
    sendCurrentsHandles_ = sendCurrentsInterface->getHandle("object2");

    return true;
}

void MyCustomController::starting(const ros::Time& time) {
    
    r_i<<1,0,0;
    t=0;
    state.dq.resize(6,1);
    state.ddq.resize(6,1);
    robot_leader.initial(r_i);
    robot_follower.initial(r_i);
    last_time=time;
    tau.resize(4,1);

    //ROS_WARN_STREAM(" success pass update");
}

void MyCustomController::stopping(const ros::Time& time) {
    std::array<double, NUM_WHEELS> currents = {0,0,0,0};
    sendCurrentsHandles_.setCommand(currents[0], currents[1], currents[2], currents[3]);
}

void MyCustomController::update(const ros::Time& time, const ros::Duration& period) {
    t=t+period.toSec();
    //t=time.toSec()-last_time.toSec();
    //last_time=time;

    //根据高级控制算法编写
    state.x[0]        = motionCaptureHandles_.getPosition1()/1000;
    state.x[1]        = motionCaptureHandles_.getPosition2()/1000;
    state.x[2]        = 0;
    state.v[0]        = motionCaptureHandles_.getVelocity1()/1000;
    state.v[1]        = motionCaptureHandles_.getVelocity2()/1000;
    state.v[2]        = 0;
    state.w[0]        = 0;
    state.w[1]        = 0;
    state.w[2]        = motionCaptureHandles_.getAngularVelocity()/1000;

    //旋转矩阵待定
    qua.w() = motionCaptureHandles_.getAttitude2();
    qua.x() = 0;
    qua.y() = 0;
    qua.z() = motionCaptureHandles_.getAttitude1();
    state.R = qua.toRotationMatrix();

    double theta = std::atan2(state.R(1, 0), state.R(0, 0));
    
    // 将角度从弧度转换为度
    double theta_deg = theta * 180.0 / M_PI;
    //ROS_WARN_STREAM("state.R: " << state.R << ", success pass update");
    //ROS_WARN_STREAM("theta_deg: " << theta_deg << ", success pass update");

    

    // state.R<<cos(motionCaptureHandles_.getAttitude()),-sin(motionCaptureHandles_.getAttitude()),0,
    //          sin(motionCaptureHandles_.getAttitude()),cos(motionCaptureHandles_.getAttitude()),0,
    //          0,0,1;
    //C
    target.x_d<<1*sin(t),4*(-cos(t)),0;
    target.v_d<<1*cos(t),4*(sin(t)),0;
    target.a_d<<1*(-sin(t)),4*(cos(t)),0;
    target.w_d<<0,0,0.1*cos(t);
    target.al_d<<0,0,0.1*(-sin(t));

    robot_leader.robotupdate(target,state,period.toSec());
    tau=robot_leader.tau;


    // 拼接变换后的前三维和后三维
    //ROS_WARN_STREAM("before: " << tau << ", success pass update");
    tau << state.R.transpose()*tau.head<3>(), tau.tail<3>();
    //ROS_WARN_STREAM("later: " << tau << ", success pass update");


    //robot_follower.robotupdate(target.w_d,target.a_d,target.al_d,state,period.toSec(),t);
    //tau=robot_follower.tau;

    std::array<double, NUM_WHEELS> currents = calculateCurrents(tau);
//    std::array<double, NUM_WHEELS> currents = {0,0,0,2};
    // ROS_WARN_STREAM("tau[0]: " << tau[0] << ", success pass update");
    // ROS_WARN_STREAM("tau[1]: " << tau[1] << ", success pass update");
    // ROS_WARN_STREAM("tau[2]: " << tau[2] << ", success pass update");
    // ROS_WARN_STREAM("tau[3]: " << tau[3] << ", success pass update");
    // ROS_WARN_STREAM("tau[4]: " << tau[4] << ", success pass update");
    // ROS_WARN_STREAM("tau[5]: " << tau[5] << ", success pass update");
    ROS_WARN_STREAM("currents[0]: " << currents[0] << ", success pass update");
    ROS_WARN_STREAM("currents[1]: " << currents[1] << ", success pass update");
    ROS_WARN_STREAM("currents[2]: " << currents[2] << ", success pass update");
    ROS_WARN_STREAM("currents[3]: " << currents[3] << ", success pass update");
    if (currents[0]<10 && currents[1]<10 && currents[2]<10 && currents[3]<10){
        sendCurrentsHandles_.setCommand(currents[0]*819.2, currents[2]*819.2, -currents[3]*819.2, -currents[1]*819.2);
    }
    else{
        sendCurrentsHandles_.setCommand(-10*819.2, 10*819.2, -10*819.2, 10*819.2);
        ROS_WARN_STREAM("Boom!");
    }
}


} // namespace CRBT


PLUGINLIB_EXPORT_CLASS(CRBT::MyCustomController, controller_interface::ControllerBase)

