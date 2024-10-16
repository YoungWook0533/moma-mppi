#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp> 
#include <pinocchio/parsers/urdf.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>


class VelocityToEffortConverter {
public:
    VelocityToEffortConverter() {
        
        std::string file_name = "/home/kist/moma_mppi_ws/src/moma-mppi/sampling_based_control/mppi_examples/mppi_panda/resources/panda/panda.urdf";
        pinocchio::urdf::buildModel(file_name, model);
        pinocchio::Data data(model);
        data_ = data;

        ROS_INFO("Model successfully loaded with %d joints", model.nq);
        
        q_.setZero(model.nq);
        v_.setZero(model.nv);
        q_dd_.setZero(model.nv);

        // Initialize publishers for base and effort commands
        base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 1);
        arm_effort_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

        // Subscribe to /input and /joint_states topics
        input_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/input", 1, &VelocityToEffortConverter::inputCallback, this);
        joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &VelocityToEffortConverter::jointStateCallback, this);

        // Initialize PID controllers for each joint (7 joints for Panda)
        for (int i = 0; i < 7; ++i) {
            control_toolbox::Pid pid;
            switch (i) {
                case 0:
                    pid.initPid(120.0, 0.01, 0.5, 100.0, -100.0);
                    break;
                case 1:
                    pid.initPid(300.0, 0.05, 1.0, 100.0, -100.0);
                    break;
                case 2:
                    pid.initPid(180.0, 0.01, 0.5, 100.0, -100.0);
                    break;
                case 3:
                    pid.initPid(180.0, 0.01, 0.7, 100.0, -100.0);
                    break;
                case 4:
                    pid.initPid(120.0, 0.01, 0.7, 100.0, -100.0);
                    break;
                case 5:
                    pid.initPid(70.0, 0.01, 0.5, 100.0, -100.0);
                    break;
                case 6:
                    pid.initPid(10.0, 0.01, 0.02, 100.0, -100.0);
                    break;
            }
            pid_controllers_.push_back(pid);
        }
    }

    void inputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 11) {
            ROS_WARN_STREAM("Received input size is incorrect. Expected 11 elements.");
            return;
        }

        // Extract the base twist (first 3 elements for linear.x, linear.y, and angular.z)
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = msg->data[0];
        base_cmd.linear.y = msg->data[1];
        base_cmd.angular.z = msg->data[2];
        base_cmd_pub_.publish(base_cmd);  // Publish the base twist for the mobile base

        // Extract the desired joint velocities (next 7 elements for the manipulator)
        desired_velocities_.clear();
        for (int i = 0; i < 7; ++i) {
            desired_velocities_.push_back(msg->data[3 + i]);
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        ros::Time start_time = ros::Time::now();

        if (msg->position.size() < 7 || msg->velocity.size() < 7) {
            ROS_WARN_STREAM("Joint state size is less than expected.");
            return;
        }

        // Update the current joint positions and velocities from the message
        for (int i = 0; i < 7; ++i) {
            q_[i] = msg->position[i];
            v_[i] = msg->velocity[i];
        }

        // Compute the joint accelerations using PID control
        for (int i = 0; i < 7; ++i) {
            double velocity_error = desired_velocities_[i] - msg->velocity[i];
            q_dd_[i] = pid_controllers_[i].computeCommand(velocity_error, ros::Duration(0.001));
        }

        pinocchio::computeAllTerms(model, data_, q_, v_);
        Eigen::VectorXd tau = data_.M * q_dd_+ data_.nle;
        std_msgs::Float64MultiArray effort_cmd;
        effort_cmd.data.resize(7);

        for (int i = 0; i < 7; ++i) {
            effort_cmd.data[i] = tau[i];
        }

        // Publish the effort commands
        ros::Duration computation_duration = ros::Time::now() - start_time;

        if (computation_duration.toSec() > 0.001) {
            arm_effort_pub_.publish(previous_effort_cmd_);
        } else {
            arm_effort_pub_.publish(effort_cmd);
            previous_effort_cmd_ = effort_cmd;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher base_cmd_pub_;
    ros::Publisher arm_effort_pub_;
    ros::Subscriber input_sub_;
    ros::Subscriber joint_state_sub_;

    std::vector<double> desired_velocities_;
    std::vector<control_toolbox::Pid> pid_controllers_;
    std_msgs::Float64MultiArray previous_effort_cmd_;

    pinocchio::Model model;
    pinocchio::Data data_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd q_dd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_to_effort_converter");

    VelocityToEffortConverter converter;

    ros::spin();
    return 0;
}
