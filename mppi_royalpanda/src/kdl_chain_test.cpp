#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kdl_chain_test");
  ros::NodeHandle nh;

  // Load URDF from parameter server
  std::string robot_description_string;
  nh.getParam("/robot_description", robot_description_string);

  urdf::Model model;
  if (!model.initString(robot_description_string)) {
    ROS_ERROR("Failed to parse URDF file");
    return -1;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to construct KDL tree from URDF");
    return -1;
  }

  // Extract the kinematic chain from world to panda_hand
  KDL::Chain chain;
  if (!tree.getChain("base_link", "panda_hand", chain)) {
    ROS_ERROR("Failed to extract chain from base_link to panda_hand");
    return -1;
  }

  ROS_INFO("Successfully extracted chain from world to panda_hand");
  ROS_INFO("Number of segments in the chain: %lu", chain.getNrOfSegments());
  ROS_INFO("Number of joints in the chain: %lu", chain.getNrOfJoints());

  return 0;
}
