#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_publisher_node");
    ros::NodeHandle nh("~");

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/arm_leader_controller/command_pose", 1);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/arm_leader_controller/command_twist", 1);

    // Subscriber to get the initial position from the controller
    // We use a latching-like behavior: read once to set the center
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

    nh.param("freq", freq, 0.5);
    nh.param("radius", radius, 0.1);

    ROS_INFO("Waiting for initial robot pose...");
    while (ros::ok() && !initial_pose_found) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Starting Circle Trajectory Publisher.");
    ROS_INFO("Center: [%.2f, %.2f, %.2f], Radius: %.2f", center_x, center_y, center_z, radius);
    
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
        double y_ref = center_y + current_radius * sin(omega * t);
        double z_ref = center_z + current_radius * cos(omega * t);

        double vx_ref = 0.0;
        double vy_ref = current_radius * omega * cos(omega * t);
        double vz_ref = -current_radius * omega * sin(omega * t);
        
        // Add ramp derivative to velocity (optional but more correct)
        if (t < 5.0) {
             double dr = radius / 5.0;
             y_ref = center_y + current_radius * sin(omega * t);
             z_ref = center_z + current_radius * cos(omega * t);
             // v = dr/dt * pos_unit + r * d(pos_unit)/dt
             vy_ref = dr * sin(omega * t) + current_radius * omega * cos(omega * t);
             vz_ref = dr * cos(omega * t) - current_radius * omega * sin(omega * t);
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

        pose_pub.publish(pose_msg);
        twist_pub.publish(twist_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
