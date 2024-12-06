// #include <pinocchio/fwd.hpp>
// #include <pinocchio/algorithm/compute-all-terms.hpp> 
// #include <pinocchio/parsers/urdf.hpp>
// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <sensor_msgs/JointState.h>
// #include <control_toolbox/pid.h>

// class VelocityToEffortConverter {
// public:
//     VelocityToEffortConverter() {
//         std::string file_name = "/home/kist/moma_mppi_ws/src/moma-mppi/sampling_based_control/mppi_examples/mppi_panda/resources/panda/panda.urdf";
//         pinocchio::urdf::buildModel(file_name, model);
//         pinocchio::Data data(model);
//         data_ = data;

//         ROS_INFO("Model successfully loaded with %d joints", model.nq);
        
//         q_.setZero(model.nq);
//         v_.setZero(model.nv);
//         q_dd_.setZero(model.nv);

//         // Initialize publishers for base and effort commands
//         base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 1);
//         arm_effort_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

//         // Subscribe to /input and /joint_states topics
//         input_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/input", 1, &VelocityToEffortConverter::inputCallback, this);
//         joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &VelocityToEffortConverter::jointStateCallback, this);

//         previous_effort_cmd_.data.resize(7, 0.0);  // Initialize previous effort
//         effort_computed_in_time_ = false;
//     }

//     void inputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//         try {
//             if (msg->data.size() != 11) {
//                 ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
//                 return;
//             }

//             // Extract the base twist (first 3 elements for linear.x, linear.y, and angular.z)
//             geometry_msgs::Twist base_cmd;
//             base_cmd.linear.x = msg->data[0];
//             base_cmd.linear.y = msg->data[1];
//             base_cmd.angular.z = msg->data[2];
//             base_cmd_pub_.publish(base_cmd);  // Publish the base twist for the mobile base

//             // Extract the desired joint velocities (next 7 elements for the manipulator)
//             desired_velocities_.clear();
//             for (int i = 0; i < 7; ++i) {
//                 desired_velocities_.push_back(msg->data[3 + i]);
//             }
//         } catch (const std::exception& e) {
//             ROS_ERROR("Error in inputCallback: %s", e.what());
//         }
//     }

//     void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//         if (msg->position.size() < 7 || msg->velocity.size() < 7) {
//             ROS_WARN("Joint state size is less than expected.");
//             return;
//         }

//         if (desired_velocities_.size() < 7) {
//             ROS_WARN("Desired velocities are not initialized. Skipping callback.");
//             return;
//         }

//         // Update joint states
//         for (int i = 0; i < 7; ++i) {
//             q_[i] = msg->position[i];
//             v_[i] = msg->velocity[i];
//         }

       

        
//         for (int i = 0; i < 7; ++i) {
//             q_dd_[i] = (desired_velocities_[i] - v_[i]) / delta_time.toSec();
//         }
        
        

//         pinocchio::computeAllTerms(model, data_, q_, v_);

//         Eigen::VectorXd tau = data_.M * q_dd_ + data_.nle;

//         for (int i = 0; i < 7; ++i) {
//             previous_effort_cmd_.data[i] = tau[i];
//         }
//         effort_computed_in_time_ = true;
//     }


//     void publishEffort() {
//         try {
//             // Publish the current effort command if computed successfully, else publish the previous effort command
//             if (effort_computed_in_time_) {
//                 arm_effort_pub_.publish(previous_effort_cmd_);
//                 effort_computed_in_time_ = false;  // Reset the flag for the next cycle
//             } else {
//                 arm_effort_pub_.publish(previous_effort_cmd_);  // Publish previous effort if current is not ready
//             }
//         } catch (const std::exception& e) {
//             ROS_ERROR("Error in publishEffort: %s", e.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Publisher base_cmd_pub_;
//     ros::Publisher arm_effort_pub_;
//     ros::Subscriber input_sub_;
//     ros::Subscriber joint_state_sub_;

//     std::vector<double> desired_velocities_;
//     std::vector<control_toolbox::Pid> pid_controllers_;
//     std_msgs::Float64MultiArray previous_effort_cmd_;
//     bool effort_computed_in_time_;  // Flag to track whether effort was computed in time

//     pinocchio::Model model;
//     pinocchio::Data data_;

//     Eigen::VectorXd q_;
//     Eigen::VectorXd v_;
//     Eigen::VectorXd q_dd_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "velocity_to_effort_converter");

//     VelocityToEffortConverter converter;

//     // Set the control loop rate to 1000 Hz
//     ros::Rate loop_rate(100);

//     while (ros::ok()) {
//         ros::spinOnce();

//         // Publish effort commands at 1000Hz
//         converter.publishEffort();

//         loop_rate.sleep();
//     }

//     return 0;
// }


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "input_to_controllers");
    ros::NodeHandle nh;

    // Publishers for the base and arm controllers
    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 1);
    ros::Publisher arm_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

    // Create variables to store the previous commands for the base and arm
    geometry_msgs::Twist previous_base_cmd;
    previous_base_cmd.linear.x = 0.0;
    previous_base_cmd.linear.y = 0.0;
    previous_base_cmd.angular.z = 0.0;

    std_msgs::Float64MultiArray previous_arm_cmd;
    previous_arm_cmd.data.resize(7, 0.0);  // Initialize to zeros

    // Smoothing factors for exponential smoothing
    const double base_alpha = 1.0;  // Smoothing factor for base commands
    const double joint_alpha = 1.0; // Smoothing factor for joint velocities

    // Flag indicating whether a new command has been received
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

        // Apply exponential smoothing to base commands
        base_cmd.linear.x = base_alpha * base_cmd.linear.x + (1 - base_alpha) * previous_base_cmd.linear.x;
        base_cmd.linear.y = base_alpha * base_cmd.linear.y + (1 - base_alpha) * previous_base_cmd.linear.y;
        base_cmd.angular.z = base_alpha * base_cmd.angular.z + (1 - base_alpha) * previous_base_cmd.angular.z;

        // Publish the smoothed base command
        base_cmd_pub.publish(base_cmd);

        // Update the previous base command with the newly calculated command
        previous_base_cmd = base_cmd;

        // Extract the joint velocities (next 7 elements for the manipulator)
        std_msgs::Float64MultiArray arm_cmd;
        arm_cmd.data.resize(7);
        for (int i = 0; i < 7; ++i) {
            arm_cmd.data[i] = msg->data[3 + i];  // Populate the joint velocities
        }

        // Apply exponential smoothing to arm joint velocities
        for (int i = 0; i < 7; ++i) {
            arm_cmd.data[i] = joint_alpha * arm_cmd.data[i] + (1 - joint_alpha) * previous_arm_cmd.data[i];
        }

        // Publish the smoothed arm command
        arm_cmd_pub.publish(arm_cmd);

        // Update the previous arm command with the newly calculated command
        previous_arm_cmd = arm_cmd;

        // Mark that a new command has been received
        new_command_received = true;
    };

    // Subscribe to /input topic
    ros::Subscriber input_sub = nh.subscribe<std_msgs::Float32MultiArray>("/input", 1, input_cb);

    // Set the control loop rate to 1000Hz
    ros::Rate loop_rate(1000);

    while (ros::ok()) {
        // If a new command has been received, publish the new command; otherwise, publish the previous command
        if (new_command_received) {
            // Reset the flag for the next loop
            new_command_received = false;
        }

        // Spin to handle callbacks
        ros::spinOnce();

        // Sleep to maintain the 1000Hz rate
        loop_rate.sleep();
    }

    return 0;
}
