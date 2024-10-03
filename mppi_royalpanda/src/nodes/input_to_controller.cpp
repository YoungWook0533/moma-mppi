#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>  // Message type of /input topic

int main(int argc, char** argv) {
    ros::init(argc, argv, "input_to_controllers");
    ros::NodeHandle nh;

    // Publishers for the base and arm controllers
    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);
    ros::Publisher arm_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

    // Subscriber callback for /input topic
    auto input_cb = [&](const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 11) {
            ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
            return;
        }

        // Extract the base twist (first 3 elements for linear.x, linear.y, and angular.z)
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = msg->data[0];
        base_cmd.linear.y = msg->data[1];
        base_cmd.angular.z = msg->data[2];
        base_cmd_pub.publish(base_cmd);  // Publish the base twist

        // Extract the joint positions (next 7 elements for the manipulator)
        std_msgs::Float64MultiArray arm_cmd;
        arm_cmd.data.resize(7);
        for (int i = 0; i < 7; ++i) {
            arm_cmd.data[i] = msg->data[3 + i];  // Populate the joint positions
        }

        arm_cmd_pub.publish(arm_cmd);  // Publish the joint positions for the arm
    };

    // Subscribe to the /input topic
    ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);

    ros::spin();  // Keep the node running
    return 0;
}
