#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <string>

// Global config
std::string OBJECT_NAME = "shared_block";
std::string OUTPUT_TOPIC = "/shared_object/state";

ros::Publisher pub_odom;

void gazeboStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
    auto it = std::find(msg->name.begin(), msg->name.end(), OBJECT_NAME);
    if (it != msg->name.end()) {
        int index = std::distance(msg->name.begin(), it);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = OBJECT_NAME;

        // Pose
        odom.pose.pose = msg->pose[index];

        // Twist (Linear & Angular Velocity)
        odom.twist.twist = msg->twist[index];

        // Covariance (Optional, setting to zero or identity as needed)
        // For now, we leave them as zero since Gazebo is "ground truth"

        pub_odom.publish(odom);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_state_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("object_name", OBJECT_NAME, "shared_block");
    pnh.param<std::string>("output_topic", OUTPUT_TOPIC, "/shared_object/state");

    pub_odom = nh.advertise<nav_msgs::Odometry>(OUTPUT_TOPIC, 1);
    ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1, gazeboStatesCallback);

    ROS_INFO("Started Object State Publisher for '%s' -> '%s'", OBJECT_NAME.c_str(), OUTPUT_TOPIC.c_str());

    ros::spin();
    return 0;
}
