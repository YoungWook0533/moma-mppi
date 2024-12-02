#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>

std::vector<double> computeSavitzkyGolayCoefficients(int window_size, int polynomial_order) {
    std::vector<double> coefficients(window_size, 0.0);
    int half_window = window_size / 2;

    Eigen::MatrixXd A(window_size, polynomial_order + 1);
    Eigen::VectorXd b(window_size);

    for (int i = -half_window; i <= half_window; ++i) {
        for (int j = 0; j <= polynomial_order; ++j) {
            A(i + half_window, j) = std::pow(i, j);
        }
        b(i + half_window) = (i == 0) ? 1.0 : 0.0;
    }

    Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    for (int i = 0; i < window_size; ++i) {
        coefficients[i] = coeffs[i];
    }

    return coefficients;
}

// Apply Savitzky-Golay filter to a sliding window of data (convolution)
std::vector<double> applySavitzkyGolayFilter(const std::deque<std::vector<double>>& data, const std::vector<double>& coefficients) {
    size_t num_inputs = data.front().size();
    std::vector<double> smoothed_data(num_inputs, 0.0);

    for (size_t j = 0; j < num_inputs; ++j) {
        for (size_t i = 0; i < data.size(); ++i) {
            smoothed_data[j] += data[i][j] * coefficients[i];
        }
    }

    return smoothed_data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "input_to_controllers");
    ros::NodeHandle nh;

    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/robot/robotnik_base_control/cmd_vel", 1);
    ros::Publisher arm_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/position_joint_trajectory_controller/command", 1);

    // For maintaining a sliding window of inputs
    std::deque<std::vector<double>> input_window;

    // Input data buffers
    std::vector<double> joint_positions(7, 0.0);
    std::vector<double> joint_velocities(7, 0.0);
    std::vector<double> previous_desired_joint(7, 0.0);

    std::vector<double> joint_position_min = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
    std::vector<double> joint_position_max = {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

    bool first_time = true;
    ros::Time last_update_time = ros::Time::now();

    // Savitzky-Golay parameters
    int window_size = 5;  // Must be odd
    int polynomial_order = 2;
    std::vector<double> sg_coefficients = computeSavitzkyGolayCoefficients(window_size, polynomial_order);

    auto joint_states_cb = [&](const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_positions[i] = msg->position[i];
        }
    };

    auto input_cb = [&](const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 11) {
            ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
            return;
        }

        // Add inputs to the sliding window
        input_window.push_back(msg->data);
        if (input_window.size() > (size_t)window_size) {
            input_window.pop_front();
        }
    };

    ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);
    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/panda/joint_states", 1, joint_states_cb);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // Ensure sufficient data is available for filtering
        if (input_window.size() < (size_t)window_size) {
            continue;
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time).toSec();
        last_update_time = current_time;

        // Apply Savitzky-Golay filter to all inputs
        std::vector<double> smoothed_inputs = applySavitzkyGolayFilter(input_window, sg_coefficients);

        // Separate base and manipulator inputs
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = smoothed_inputs[0];
        base_cmd.linear.y = smoothed_inputs[1];
        base_cmd.angular.z = smoothed_inputs[2];
        base_cmd_pub.publish(base_cmd);

        std::vector<double> current_joint_positions(7, 0.0);
        for (size_t i = 0; i < 7; ++i) {
            joint_velocities[i] = smoothed_inputs[3 + i];

            // Compute joint positions
            if (first_time) {
                current_joint_positions[i] = joint_positions[i] + joint_velocities[i] * dt;
            } else {
                current_joint_positions[i] = previous_desired_joint[i] + joint_velocities[i] * dt;
            }

            current_joint_positions[i] = std::clamp(current_joint_positions[i], joint_position_min[i], joint_position_max[i]);
        }

        previous_desired_joint = current_joint_positions;
        // first_time = false;

        // Publish manipulator commands
        trajectory_msgs::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
        trajectory_msgs::JointTrajectoryPoint point;

        point.positions = current_joint_positions;
        point.time_from_start = ros::Duration(dt);

        joint_trajectory_msg.points.push_back(point);
        arm_cmd_pub.publish(joint_trajectory_msg);

        loop_rate.sleep();
    }

    return 0;
}
