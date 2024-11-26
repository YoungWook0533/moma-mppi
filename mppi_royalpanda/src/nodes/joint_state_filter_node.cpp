//
// Created by giuseppe on 09.02.21.
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_state_filter_node");
  ros::NodeHandle nh("~");
  ros::Publisher joint_state_pub =
      nh.advertise<sensor_msgs::JointState>("/panda/joint_states", 1);
  std::vector<std::string> joint_names{
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  // make sure finger comes after
  std::vector<std::string> fingers{"fr3_finger_joint1",
                                   "fr3_finger_joint2"};

  auto cb = [&](const sensor_msgs::JointStateConstPtr& msg) {
    static sensor_msgs::JointState filtered_state;
    filtered_state.name.clear();
    filtered_state.position.clear();
    filtered_state.velocity.clear();
    filtered_state.effort.clear();
    for (size_t idx = 0; idx < msg->name.size(); idx++) {
      if (std::find(joint_names.begin(), joint_names.end(), msg->name[idx]) !=
          joint_names.end()) {
        filtered_state.name.push_back(msg->name[idx]);
        filtered_state.effort.push_back(msg->effort[idx]);
        filtered_state.position.push_back(msg->position[idx]);
        filtered_state.velocity.push_back(msg->velocity[idx]);
      }
    }

    for (size_t idx = 0; idx < msg->name.size(); idx++) {
      if (std::find(fingers.begin(), fingers.end(), msg->name[idx]) !=
          fingers.end()) {
        filtered_state.name.push_back(msg->name[idx]);
        filtered_state.effort.push_back(msg->effort[idx]);
        filtered_state.position.push_back(msg->position[idx]);
        filtered_state.velocity.push_back(msg->velocity[idx]);
      }
    }

    // never send partial joints
    if (filtered_state.name.size() != (joint_names.size() + fingers.size()))
      return;

    filtered_state.header.frame_id = "odom";
    filtered_state.header.stamp = ros::Time::now();
    joint_state_pub.publish(filtered_state);
  };

  ros::Subscriber joint_sub =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, cb);
  ros::spin();
  return 0;
}