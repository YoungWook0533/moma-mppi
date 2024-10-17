#include "mppi_manipulation/controller_interface.h"
#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/conversions.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "manipulation_controller");
    ros::NodeHandle nh("~");

    ROS_INFO("Controller Node Started");

    // Define the expected size for state and control vectors
    int expected_size = 19;
    bool observation_set = false;

    // Define Eigen Vectors for state, nominal state, and control inputs
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(expected_size);
    Eigen::VectorXd x_nom_ = Eigen::VectorXd::Zero(expected_size);
    Eigen::VectorXd u_ = Eigen::VectorXd::Zero(expected_size);

    // Initialize the PandaControllerInterface
    ROS_INFO("Creating PandaControllerInterface");
    manipulation::PandaControllerInterface man_interface(nh);

    if (!man_interface.init()) {
        ROS_ERROR("Failed to initialize controller manipulation interface.");
        return 0;
    }

    ROS_INFO("Controller Interface Initialized");

    // Subscriber callback for the observer state
    auto cb = [&](const manipulation_msgs::StateConstPtr& msg) {
        double current_time = ros::Time::now().toSec();
        manipulation::conversions::msgToEigen(*msg, x_, current_time);
        man_interface.set_observation(x_, msg->header.stamp.toSec());

        // Start the controller after receiving the first observation
        if (!observation_set) {
            man_interface.start();
            ROS_INFO("[Controller Node]: First observation received and controller started!");
        }
        observation_set = true;
    };

    // Subscribe to the /observer/state topic
    ROS_INFO("Subscribing to /observer/state topic");
    ros::Subscriber state_subscriber = nh.subscribe<manipulation_msgs::State>("/observer/state", 1, cb);

    // Set up the publisher for /controller/input_state
    manipulation_msgs::InputState input_state_msg;
    ros::Publisher input_publisher = nh.advertise<manipulation_msgs::InputState>("/controller/input_state", 1);

    ros::Rate rate(100);  // Control loop rate set to 100 Hz

    // Main loop to run the controller and publish input states
    while (ros::ok()) {
        if (observation_set) {
            // Retrieve the control input and nominal state from the controller
            man_interface.get_input_state(x_, x_nom_, u_, ros::Time::now().toSec());

            // Convert Eigen vectors to ROS messages
            manipulation::conversions::eigenToMsg(u_, input_state_msg.input);
            manipulation::conversions::eigenToMsg(x_nom_, ros::Time::now().toSec(), input_state_msg.state);

            // Publish the input state
            input_publisher.publish(input_state_msg);
        } else {
            ROS_WARN_STREAM_THROTTLE(2.0, "[Controller Node]: Waiting to receive the first observation.");
        }

        ros::spinOnce();  // Handle ROS callbacks
        rate.sleep();     // Maintain loop rate
    }

    return 0;
}
