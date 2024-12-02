#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <unordered_map>
#include <algorithm>

int main(int argc, char** argv) {
    ros::init(argc, argv, "input_to_controllers");
    ros::NodeHandle nh;

    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/robot/robotnik_base_control/cmd_vel", 1);
    ros::Publisher arm_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/position_joint_trajectory_controller/command", 1);

    std::unordered_map<std::string, double> joint_positions;
    std::vector<std::string> joint_names = {
        "fr3_joint1", "fr3_joint2", "fr3_joint3",
        "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    std::vector<double> joint_velocities(7, 0.0);
    std::vector<double> smoothed_joint_velocities(7, 0.0);
    std::vector<double> smoothed_joint_accelerations(7, 0.0);

    geometry_msgs::Twist smoothed_base_cmd;  // For smoothed base commands
    double base_alpha = 0.2;  // Smoothing factor for base commands
    double joint_alpha = 0.2; // Smoothing factor for joint velocities
    double joint_beta = 0.2;  // Smoothing factor for joint accelerations

    std::vector<double> joint_position_min = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    std::vector<double> joint_position_max = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    std::vector<double> max_velocity = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> max_acceleration = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    ros::Time last_update_time = ros::Time::now();

    auto joint_states_cb = [&](const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_positions[msg->name[i]] = msg->position[i];
        }
    };

    auto input_cb = [&](const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 11) {
            ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
            return;
        }

        // Extract and smooth base commands
        smoothed_base_cmd.linear.x = base_alpha * msg->data[0] + (1 - base_alpha) * smoothed_base_cmd.linear.x;
        smoothed_base_cmd.linear.y = base_alpha * msg->data[1] + (1 - base_alpha) * smoothed_base_cmd.linear.y;
        smoothed_base_cmd.angular.z = base_alpha * msg->data[2] + (1 - base_alpha) * smoothed_base_cmd.angular.z;

        base_cmd_pub.publish(smoothed_base_cmd);

        // Update and smooth joint velocities
        for (int i = 0; i < 7; ++i) {
            joint_velocities[i] = msg->data[3 + i];
            smoothed_joint_velocities[i] = joint_alpha * joint_velocities[i] + (1 - joint_alpha) * smoothed_joint_velocities[i];
        }
    };

    ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);
    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/panda/joint_states", 1, joint_states_cb);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        std::vector<double> current_joint_positions(7, 0.0);

        // Ensure joint positions are updated with the actual robot state
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (joint_positions.find(joint_names[i]) != joint_positions.end()) {
                current_joint_positions[i] = joint_positions[joint_names[i]];
            } else {
                ROS_WARN_STREAM("Joint state for " << joint_names[i] << " not found.");
            }
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time).toSec();
        last_update_time = current_time;

        for (int i = 0; i < 7; ++i) {
            // Smooth accelerations
            double acceleration = (smoothed_joint_velocities[i] - joint_velocities[i]) / dt;
            smoothed_joint_accelerations[i] = joint_beta * acceleration + (1 - joint_beta) * smoothed_joint_accelerations[i];

            // Clamp velocity and acceleration to safe limits
            smoothed_joint_velocities[i] = std::max(-max_velocity[i], std::min(smoothed_joint_velocities[i], max_velocity[i]));
            smoothed_joint_accelerations[i] = std::max(-max_acceleration[i], std::min(smoothed_joint_accelerations[i], max_acceleration[i]));

            // Update joint positions based on smoothed velocities
            current_joint_positions[i] += smoothed_joint_velocities[i] * dt;

            // Clamp to joint limits
            current_joint_positions[i] = std::max(joint_position_min[i], std::min(current_joint_positions[i], joint_position_max[i]));
        }

        trajectory_msgs::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = joint_names;
        trajectory_msgs::JointTrajectoryPoint point;

        point.positions = current_joint_positions;
        point.time_from_start = ros::Duration(0.1);

        joint_trajectory_msg.points.push_back(point);

        arm_cmd_pub.publish(joint_trajectory_msg);

        loop_rate.sleep();
    }

    return 0;
}
