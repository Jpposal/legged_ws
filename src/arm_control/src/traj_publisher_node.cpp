#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <cmath>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_publisher_node");
    ros::NodeHandle nh("~");

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/arm_leader_controller/command_pose", 1);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/arm_leader_controller/command_twist", 1);
    ros::Publisher accel_pub = nh.advertise<geometry_msgs::AccelStamped>("/arm_leader_controller/command_accel", 1);

    // [Added] Follower publishers (Follower receives command_twist & command_accel for trajectory tracking/estimation)
    ros::Publisher pose_pub_fol = nh.advertise<geometry_msgs::PoseStamped>("/arm_follower_controller/command_pose", 1);
    ros::Publisher twist_pub_fol = nh.advertise<geometry_msgs::TwistStamped>("/arm_follower_controller/command_twist", 1);
    ros::Publisher accel_pub_fol = nh.advertise<geometry_msgs::AccelStamped>("/arm_follower_controller/command_accel", 1);

    bool initial_pose_found = false;
    double center_x = 0.6, center_y = 0.0, center_z = 0.5; // Defaults
    
    auto pose_cb = [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!initial_pose_found) {
            center_x = msg->pose.position.x;
            center_y = msg->pose.position.y;
            center_z = msg->pose.position.z;
            ROS_INFO("Received Initial Pose: [%.4f, %.4f, %.4f]", center_x, center_y, center_z);
            initial_pose_found = true;
        }
    };
    
    // Subscribe to the controller's feedback topic
    // Adjust topic name if your controller namespace is different
    ros::Subscriber sub = nh.subscribe("/leader_actual_pose", 1, (boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)>)pose_cb);

    ros::Rate loop_rate(500); 

    double freq = 0.5; // Hz
    double radius = 0.1; // meters
    std::string plane = "yz"; // Plane to draw circle in: "xy", "yz", "xz"

    // Try to load parameters from launch file
    nh.param("freq", freq, 0.5);
    nh.param("radius", radius, 0.1);
    nh.param<std::string>("plane", plane, "yz");
    
    // If center pose is provided in params, use it and don't wait for subscriber
    if (nh.getParam("center_x", center_x) && nh.getParam("center_y", center_y) && nh.getParam("center_z", center_z)) {
        ROS_INFO("Using center pose from parameters: [%.4f, %.4f, %.4f]", center_x, center_y, center_z);
        initial_pose_found = true;
    }

    ROS_INFO("Waiting for initial robot pose...");
    while (ros::ok() && !initial_pose_found) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Starting Circle Trajectory Publisher.");
    ROS_INFO("Center: [%.2f, %.2f, %.2f], Radius: %.2f, Plane: %s", center_x, center_y, center_z, radius, plane.c_str());
    
    ros::Duration(1.0).sleep(); 
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double t = (current_time - start_time).toSec();
        double omega = 2 * M_PI * freq;

        // Ramp up radius over 5 seconds to avoid jump at start
        double current_radius = radius;
        if (t < 5.0) {
            current_radius = radius * (t / 5.0);
        }

        double x_ref = center_x;
        double y_ref = center_y;
        double z_ref = center_z;
        
        double vx_ref = 0.0;
        double vy_ref = 0.0;
        double vz_ref = 0.0;

        double sin_t = sin(omega * t);
        double cos_t = cos(omega * t);

        // Ramp derivative for smooth start
        double dr = (t < 5.0) ? (radius / 5.0) : 0.0;
        
        // Acceleration Terms (assuming ramp is slow enough to ignore 2nd derivative of r)
        // ax = dd(r) * sin + 2 * dr * w * cos - r * w^2 * sin
        // Since r is linear, ddr = 0
        
        double ddr = 0.0;
        double ax_ref = 0.0, ay_ref = 0.0, az_ref = 0.0;

        if (plane == "yz") {
            y_ref = center_y + current_radius * sin_t;
            z_ref = center_z + current_radius * cos_t;
            vy_ref = dr * sin_t + current_radius * omega * cos_t;
            vz_ref = dr * cos_t - current_radius * omega * sin_t;
            
            ay_ref = 2 * dr * omega * cos_t - current_radius * omega * omega * sin_t;
            az_ref = -2 * dr * omega * sin_t - current_radius * omega * omega * cos_t;
            
        } else if (plane == "xz") {
            x_ref = center_x + current_radius * sin_t;
            z_ref = center_z + current_radius * cos_t;
            vx_ref = dr * sin_t + current_radius * omega * cos_t;
            vz_ref = dr * cos_t - current_radius * omega * sin_t;
            
            ax_ref = 2 * dr * omega * cos_t - current_radius * omega * omega * sin_t;
            az_ref = -2 * dr * omega * sin_t - current_radius * omega * omega * cos_t;
            
        } else if (plane == "xy") {
            x_ref = center_x + current_radius * sin_t;
            y_ref = center_y + current_radius * cos_t;
            vx_ref = dr * sin_t + current_radius * omega * cos_t;
            vy_ref = dr * cos_t - current_radius * omega * sin_t;
            
            ax_ref = 2 * dr * omega * cos_t - current_radius * omega * omega * sin_t;
            ay_ref = -2 * dr * omega * sin_t - current_radius * omega * omega * cos_t;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = x_ref;
        pose_msg.pose.position.y = y_ref;
        pose_msg.pose.position.z = z_ref;
        
        pose_msg.pose.orientation.w = 0.0; 
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 1.0; 
        pose_msg.pose.orientation.z = 0.0;

        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = "world";
        twist_msg.twist.linear.x = vx_ref;
        twist_msg.twist.linear.y = vy_ref;
        twist_msg.twist.linear.z = vz_ref;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        
        geometry_msgs::AccelStamped accel_msg;
        accel_msg.header.stamp = current_time;
        accel_msg.header.frame_id = "world";
        accel_msg.accel.linear.x = ax_ref;
        accel_msg.accel.linear.y = ay_ref;
        accel_msg.accel.linear.z = az_ref;
        accel_msg.accel.angular.x = 0.0;
        accel_msg.accel.angular.y = 0.0;
        accel_msg.accel.angular.z = 0.0;

        pose_pub.publish(pose_msg);
        twist_pub.publish(twist_msg);
        accel_pub.publish(accel_msg);

        pose_pub_fol.publish(pose_msg);
        twist_pub_fol.publish(twist_msg);
        accel_pub_fol.publish(accel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
