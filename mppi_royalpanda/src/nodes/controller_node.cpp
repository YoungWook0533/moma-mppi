#include "mppi_manipulation/controller_interface.h"
#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulation_controller");
  ros::NodeHandle nh("~");

  ROS_INFO("Controller Node Started");

  int expected_size = 10;  // Expected size of the state vector for base (3) + 7 arm joints
  bool observation_set = false;
  Eigen::VectorXd x_;
  Eigen::VectorXd x_nom_;
  Eigen::VectorXd u_;  // Control input vector

  // Initialize the vectors
  x_ = Eigen::VectorXd::Zero(expected_size);
  x_nom_ = Eigen::VectorXd::Zero(expected_size);
  u_ = Eigen::VectorXd::Zero(expected_size);

  ROS_INFO("Creating PandaControllerInterface");
  manipulation::PandaControllerInterface man_interface(nh);

  if (!man_interface.init()) {
    ROS_ERROR("Failed to initialize controller manipulation interface.");
    return 0;
  }

  ROS_INFO("Controller Interface Initialized");

  // Subscriber to the state topic
  auto cb = [&](const manipulation_msgs::StateConstPtr& msg) {
    ROS_INFO_STREAM("Received state message: " << *msg);

    // Check the size of the position and velocity arrays
    ROS_INFO("Position size: %lu", msg->arm_state.position.size());
    ROS_INFO("Velocity size: %lu", msg->arm_state.velocity.size());

    double current_time = ros::Time::now().toSec();
    manipulation::conversions::msgToEigen(*msg, x_, current_time);
    man_interface.set_observation(x_, msg->header.stamp.toSec());

    if (!observation_set) {
        man_interface.start();
        ROS_INFO("[Controller Node]: First observation received and controller started!");
    }
    observation_set = true;
  };

  // Subscribe to the observer state
  ros::Subscriber state_subscriber = nh.subscribe<manipulation_msgs::State>("/observer/state", 1, cb);

  // Publishers for the base and arm controllers
  ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);
  ros::Publisher arm_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/panda_position_controller/command", 1);

  manipulation_msgs::InputState input_state_msg;
  ros::Publisher input_publisher = nh.advertise<manipulation_msgs::InputState>("/controller/input_state", 1);

  // Joint trajectory message for the arm
  trajectory_msgs::JointTrajectory joint_trajectory_msg;
  joint_trajectory_msg.joint_names = {
    "panda_joint1", "panda_joint2", "panda_joint3", 
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
  };

  geometry_msgs::Twist base_cmd;

  ros::Rate rate(100);

  while (ros::ok()) {
    if (observation_set) {
      ROS_INFO("Observation received, publishing input state");

      // Get input state from the controller interface
      man_interface.get_input_state(x_, x_nom_, u_, ros::Time::now().toSec());

      // Publish base velocity commands (u_[0], u_[1], u_[2])
      base_cmd.linear.x = u_[0];
      base_cmd.linear.y = u_[1];
      base_cmd.angular.z = u_[2];
      // base_cmd_pub.publish(base_cmd);

      // Create a trajectory point for the arm
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(7);
      point.velocities.resize(7, 0.0); // Set to zero velocity (or customize if needed)
      point.time_from_start = ros::Duration(1.0);  // 1 second to reach the desired positions

      // Assign joint positions from the control input u_ (for joints 3 to 9)
      for (size_t i = 0; i < 7; i++) {
        point.positions[i] = u_[3 + i];
      }

      // Set the trajectory point in the trajectory message
      joint_trajectory_msg.points.clear();
      joint_trajectory_msg.points.push_back(point);

      // Publish the joint trajectory command
      // arm_trajectory_pub.publish(joint_trajectory_msg);

      // Publish the input state for visualization or debugging
      manipulation::conversions::eigenToMsg(u_, input_state_msg.input);
      manipulation::conversions::eigenToMsg(x_nom_, ros::Time::now().toSec(), input_state_msg.state);
      input_publisher.publish(input_state_msg);
    } else {
      ROS_WARN_STREAM_THROTTLE(2.0, "[Controller Node]: Waiting to receive the first observation.");
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
