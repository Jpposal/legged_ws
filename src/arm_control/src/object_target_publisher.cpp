#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>

// Simple Sine Wave Trajectory Publisher
// Starts IMMEDIATELY from the object (Anchor) current position.
// Ensures strict CoM trajectory for both arms.

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_publisher_node");
    ros::NodeHandle nh("~");

    // Publishers
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/arm_leader_controller/command_pose", 1);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/arm_leader_controller/command_twist", 1);
    ros::Publisher accel_pub = nh.advertise<geometry_msgs::AccelStamped>("/arm_leader_controller/command_accel", 1);
    
    ros::Publisher pose_pub_fol = nh.advertise<geometry_msgs::PoseStamped>("/arm_follower_controller/command_pose", 1);
    ros::Publisher twist_pub_fol = nh.advertise<geometry_msgs::TwistStamped>("/arm_follower_controller/command_twist", 1);
    ros::Publisher accel_pub_fol = nh.advertise<geometry_msgs::AccelStamped>("/arm_follower_controller/command_accel", 1);

    // Transform / State
    geometry_msgs::Pose initial_pose;
    bool initial_pose_found = false;

    // Parameters
    double freq = 0.5; 
    double radius = 0.1; 
    std::string plane = "yz";
    
    // We strictly use these to define the plane, but the CENTER is strictly calculated from start pose.
    nh.param("radius", radius, 0.1);
    nh.param("freq", freq, 0.5);
    nh.param<std::string>("plane", plane, "yz");

    // Explicit Center Parameters
    double param_center_x, param_center_y, param_center_z;
    nh.param("center_x", param_center_x, 0.0);
    nh.param("center_y", param_center_y, 0.0);
    nh.param("center_z", param_center_z, 0.9);

    std::string object_name;
    nh.param<std::string>("object_name", object_name, "shared_block");
    ROS_INFO_STREAM("Trajectory Publisher waiting for object: " << object_name);

    // Subscriber for Initial Pose (From Gazebo Truth via Odometry Bridge)
    auto pose_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        if (initial_pose_found) return;

        // Directly capture the pose from the ground truth topic
        initial_pose = msg->pose.pose;
        ROS_INFO("Received Initial Anchor/CoM Pose: [%.4f, %.4f, %.4f]", 
                    initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        initial_pose_found = true;
    };
    ros::Subscriber sub = nh.subscribe("/shared_object/state", 1, (boost::function<void(const nav_msgs::Odometry::ConstPtr&)>)pose_cb);

    ros::Rate loop_rate(500);
    
    ROS_INFO("Waiting for initial robot pose...");
    while (ros::ok() && !initial_pose_found) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    double start_x = initial_pose.position.x;
    double start_y = initial_pose.position.y;
    double start_z = initial_pose.position.z;
    
    // User authorized Step Input: Use parameters directly
    double center_x = param_center_x;
    double center_y = param_center_y;
    double center_z = param_center_z;

    ROS_INFO("Calculated Trajectory Center: [%.4f, %.4f, %.4f] based on Start Pose [%.4f, %.4f, %.4f]", 
             center_x, center_y, center_z, start_x, start_y, start_z);
    ROS_INFO("Starting IMMEDIATE Sine Wave...");

    // Removed artificial delay
    // ros::Duration(1.0).sleep(); 
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double t = (current_time - start_time).toSec();
        double omega = 2 * M_PI * freq;
        
        double sin_t = sin(omega * t);
        double cos_t = cos(omega * t);

        // Constant Radius (Standard Sine Wave)
        double current_radius = radius;
        double dr = 0.0;
        
        double x_ref = center_x;
        double y_ref = center_y;
        double z_ref = center_z;
        double vx_ref = 0, vy_ref = 0, vz_ref = 0;
        double ax_ref = 0, ay_ref = 0, az_ref = 0;

        if (plane == "yz") {
            // Circle Trajectory
            y_ref = -0.05 + current_radius * sin_t; // center_y is -0.05
            z_ref = center_z + current_radius * cos_t;
            
            vy_ref = current_radius * omega * cos_t;
            vz_ref = -current_radius * omega * sin_t;
            
            ay_ref = -current_radius * omega * omega * sin_t;
            az_ref = -current_radius * omega * omega * cos_t;
        } 
        else if (plane == "xz") {
            x_ref = center_x + current_radius * sin_t;
            z_ref = center_z + current_radius * cos_t;
            
            vx_ref = current_radius * omega * cos_t;
            vz_ref = -current_radius * omega * sin_t;
            
            ax_ref = -current_radius * omega * omega * sin_t;
            az_ref = -current_radius * omega * omega * cos_t;
        }
        else if (plane == "xy") { 
            x_ref = center_x + current_radius * sin_t;
            y_ref = center_y + current_radius * cos_t;
            
            vx_ref = current_radius * omega * cos_t;
            vy_ref = -current_radius * omega * sin_t;
            
            ax_ref = -current_radius * omega * omega * sin_t;
            ay_ref = -current_radius * omega * omega * cos_t;
        }

        // Publish
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = x_ref;
        pose_msg.pose.position.y = y_ref;
        pose_msg.pose.position.z = z_ref;
        pose_msg.pose.orientation = initial_pose.orientation;

        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = "world";
        twist_msg.twist.linear.x = vx_ref;
        twist_msg.twist.linear.y = vy_ref;
        twist_msg.twist.linear.z = vz_ref;

        geometry_msgs::AccelStamped accel_msg;
        accel_msg.header.stamp = current_time;
        accel_msg.accel.linear.x = ax_ref;
        accel_msg.accel.linear.y = ay_ref;
        accel_msg.accel.linear.z = az_ref;

        pose_pub.publish(pose_msg);
        twist_pub.publish(twist_msg);
        accel_pub.publish(accel_msg);
        
        pose_pub_fol.publish(pose_msg);
        twist_pub_fol.publish(twist_msg);
        accel_pub_fol.publish(accel_msg);

        loop_rate.sleep();
    }
    return 0;
}
