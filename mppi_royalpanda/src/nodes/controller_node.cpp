#include "mppi_manipulation/controller_interface.h"
#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/conversions.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulation_controller");
  ros::NodeHandle nh("~");

  ROS_INFO("Controller Node Started");

  int expected_size = 19;
  bool observation_set = false;
  Eigen::VectorXd x_;
  Eigen::VectorXd x_nom_;
  Eigen::VectorXd u_;

  ROS_INFO("Initializing Eigen Vectors");
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

  auto cb = [&](const manipulation_msgs::StateConstPtr& msg) {
    // ROS_INFO_STREAM("Received state message: " << *msg);

    // // Check the size of the position and velocity arrays
    // ROS_INFO("Position size: %lu", msg->arm_state.position.size());
    // ROS_INFO("Velocity size: %lu", msg->arm_state.velocity.size());
    
    double current_time = ros::Time::now().toSec();
    manipulation::conversions::msgToEigen(*msg, x_, current_time);
    man_interface.set_observation(x_, msg->header.stamp.toSec());
    
    if (!observation_set) {
        man_interface.start();
        ROS_INFO("[Controller Node]: First observation received and controller started!");
    }
    observation_set = true;
};


  ROS_INFO("Subscribing to /observer/state topic");
  ros::Subscriber state_subscriber = nh.subscribe<manipulation_msgs::State>("/observer/state", 1, cb);

  manipulation_msgs::InputState input_state_msg;
  ros::Publisher input_publisher = nh.advertise<manipulation_msgs::InputState>("/controller/input_state", 1);

  ros::Rate rate(100);

  while (ros::ok()) {
    if (observation_set) {
      // ROS_INFO("Observation received, publishing input state");
      man_interface.get_input_state(x_, x_nom_, u_, ros::Time::now().toSec());

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