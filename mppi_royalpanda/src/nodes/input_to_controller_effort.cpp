// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <vector>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "input_to_controllers");
//     ros::NodeHandle nh;

//     // Publishers for the base and arm controllers
//     ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 1);
//     ros::Publisher arm_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

//     // Create variables to store the previous arm commands
//     std_msgs::Float64MultiArray previous_arm_cmd;
//     previous_arm_cmd.data.resize(7, 0.0);  // Initialize to zeros

//     // Create variable to store the flag indicating whether a new command has been received
//     bool new_command_received = false;

//     // Subscriber callback for /input topic
//     auto input_cb = [&](const std_msgs::Float32MultiArray::ConstPtr& msg) {
//         if (msg->data.size() != 11) {
//             ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
//             return;
//         }

//         // Extract the base twist (first 3 elements for linear.x, linear.y, and angular.z)
//         geometry_msgs::Twist base_cmd;
//         base_cmd.linear.x = msg->data[0];
//         base_cmd.linear.y = msg->data[1];
//         base_cmd.angular.z = msg->data[2];
//         base_cmd_pub.publish(base_cmd);  // Publish the base twist

//         // Extract the joint positions (next 7 elements for the manipulator)
//         std_msgs::Float64MultiArray arm_cmd;
//         arm_cmd.data.resize(7);
//         for (int i = 0; i < 7; ++i) {
//             arm_cmd.data[i] = msg->data[3 + i];  // Populate the joint positions
//         }

//         // Update the previous command with the newly calculated command
//         previous_arm_cmd = arm_cmd;

//         // Mark that a new command has been received
//         new_command_received = true;
//     };

//     // Subscribe to /input
//     ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);

//     // Set the control loop rate to 1000Hz
//     ros::Rate loop_rate(1000);

//     while (ros::ok()) {
//         // If a new command has been received, publish the new command; otherwise, publish the previous command
//         if (new_command_received) {
//             arm_cmd_pub.publish(previous_arm_cmd);  // Publish the updated arm command
//             new_command_received = false;  // Reset the flag for the next loop
//         } else {
//             arm_cmd_pub.publish(previous_arm_cmd);  // Publish the previous arm command
//         }

//         // Spin to handle callbacks
//         ros::spinOnce();

//         // Sleep to maintain the 1000Hz rate
//         loop_rate.sleep();
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "input_to_controllers");
    ros::NodeHandle nh;

    // Publishers for the base and each joint velocity controllers
    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel1", 1);
    ros::Publisher joint1_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint1_controller/command", 1);
    ros::Publisher joint2_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint2_controller/command", 1);
    ros::Publisher joint3_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint3_controller/command", 1);
    ros::Publisher joint4_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint4_controller/command", 1);
    ros::Publisher joint5_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint5_controller/command", 1);
    ros::Publisher joint6_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint6_controller/command", 1);
    ros::Publisher joint7_cmd_pub = nh.advertise<std_msgs::Float64>("/panda_joint7_controller/command", 1);

    // Store the previous velocity commands for each joint
    std::vector<std_msgs::Float64> previous_joint_cmd(7);
    for (auto& cmd : previous_joint_cmd) {
        cmd.data = 0.0;  // Initialize to zero
    }

    bool new_command_received = false;

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

        // Extract the joint velocities (next 7 elements for the manipulator)
        for (int i = 0; i < 7; ++i) {
            previous_joint_cmd[i].data = msg->data[3 + i];  // Update the joint velocity
        }

        new_command_received = true;
    };

    // Subscribe to /input
    ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);

    // Set the control loop rate to 1000Hz
    ros::Rate loop_rate(1000);

    while (ros::ok()) {
        // Publish velocity commands for each joint
        if (new_command_received) {
            joint1_cmd_pub.publish(previous_joint_cmd[0]);
            joint2_cmd_pub.publish(previous_joint_cmd[1]);
            joint3_cmd_pub.publish(previous_joint_cmd[2]);
            joint4_cmd_pub.publish(previous_joint_cmd[3]);
            joint5_cmd_pub.publish(previous_joint_cmd[4]);
            joint6_cmd_pub.publish(previous_joint_cmd[5]);
            joint7_cmd_pub.publish(previous_joint_cmd[6]);

            new_command_received = false;
        } else {
            // Continue publishing the previous velocity commands
            joint1_cmd_pub.publish(previous_joint_cmd[0]);
            joint2_cmd_pub.publish(previous_joint_cmd[1]);
            joint3_cmd_pub.publish(previous_joint_cmd[2]);
            joint4_cmd_pub.publish(previous_joint_cmd[3]);
            joint5_cmd_pub.publish(previous_joint_cmd[4]);
            joint6_cmd_pub.publish(previous_joint_cmd[5]);
            joint7_cmd_pub.publish(previous_joint_cmd[6]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
