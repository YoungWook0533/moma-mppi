#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>

class VelocityToEffortConverter {
public:
    VelocityToEffortConverter() {
        // Initialize publishers for base and effort commands
        base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 1);
        arm_effort_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/panda_position_controller/command", 1);

        // Subscribe to the /input topic
        input_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/input", 1, &VelocityToEffortConverter::inputCallback, this);

        // Subscribe to joint states to get current joint velocities
        joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &VelocityToEffortConverter::jointStateCallback, this);

        // Initialize PID controllers for each joint (7 joints for Panda)
        for (int i = 0; i < 7; ++i) {
            control_toolbox::Pid pid;
            pid.initPid(100.0, 0.1, 10.0, 100.0, -100.0);  // You can tune these values
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
        base_cmd_pub_.publish(base_cmd);  // Publish the base twist

        // Extract the desired joint velocities (next 7 elements for the manipulator)
        desired_velocities_.clear();
        for (int i = 0; i < 7; ++i) {
            desired_velocities_.push_back(msg->data[3 + i]);
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->velocity.size() < 7) {
            ROS_WARN_STREAM("Joint state velocity size is less than expected.");
            return;
        }

        // Compute the effort for each joint
        std_msgs::Float64MultiArray effort_cmd;
        effort_cmd.data.resize(7);

        for (int i = 0; i < 7; ++i) {
            // Calculate velocity error
            double velocity_error = desired_velocities_[i] - msg->velocity[i];

            // Compute effort using the PID controller
            double effort = pid_controllers_[i].computeCommand(velocity_error, ros::Duration(0.01));  // 0.01 is the control loop duration (100Hz)
            effort_cmd.data[i] = effort;
        }

        // Publish the effort commands for the arm
        arm_effort_pub_.publish(effort_cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher base_cmd_pub_;
    ros::Publisher arm_effort_pub_;
    ros::Subscriber input_sub_;
    ros::Subscriber joint_state_sub_;

    std::vector<double> desired_velocities_;
    std::vector<control_toolbox::Pid> pid_controllers_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_to_effort_converter");

    VelocityToEffortConverter converter;

    ros::spin();
    return 0;
}
