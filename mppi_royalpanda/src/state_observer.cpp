//
// Created by giuseppe on 25.01.21.
//

#include "mppi_royalpanda/state_observer.h"
#include "mppi_royalpanda/utils.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <manipulation_msgs/conversions.h>

namespace manipulation_royalpanda {

StateObserver::StateObserver(const ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
  ext_tau_.setZero();
  base_twist_.setZero();
  base_pose_.setZero();

  nh_.param<bool>("simulation", simulation_, false);

  std::string arm_state_topic;
  nh_.param<std::string>("arm_state_topic", arm_state_topic, "/arm_state");

  std::string object_pose_topic;
  nh_.param<std::string>("object_pose_topic", object_pose_topic,
                         "/object_pose");

  std::string object_state_topic;
  nh_.param<std::string>("object_state_topic", object_state_topic,
                         "/object_state");

  std::string wrench_topic;
  nh_.param<std::string>("wrench_topic", wrench_topic, "/wrench");

  std::string odom_topic = "/robotnik_base_control/odom"; // Using the /Odometry topic for both pose and twist

  ROS_INFO_STREAM(
      "Subscribing arm state to: " << arm_state_topic << std::endl
                                   << "Subscribing object pose to: " << object_pose_topic << std::endl
                                   << "Subscribing object state to: " << object_state_topic << std::endl
                                   << "Subscribing wrench to: " << wrench_topic << std::endl
                                   << "Subscribing odometry to: " << odom_topic);

  // Subscribing to topics
  arm_subscriber_ = nh_.subscribe(arm_state_topic, 1, &StateObserver::arm_state_callback, this);
  object_subscriber_ = nh_.subscribe(object_pose_topic, 1, &StateObserver::object_pose_callback, this);
  wrench_subscriber_ = nh_.subscribe(wrench_topic, 1, &StateObserver::wrench_callback, this);
  object_state_subscriber_ = nh_.subscribe(object_state_topic, 1, &StateObserver::object_state_callback, this);
  odom_subscriber_ = nh_.subscribe(odom_topic, 1, &StateObserver::odom_callback, this); // Subscribe to /Odometry

  // ros publishing
  state_publisher_ = nh_.advertise<manipulation_msgs::State>("/observer/state", 1);
  base_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/observer/base_pose", 1);
  base_twist_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/observer/base_twist", 1);
  object_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 1);
  robot_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/observer/base/joint_state", 1);
  wrench_filt_publisher_ = nh_.advertise<geometry_msgs::WrenchStamped>("/observer/wrench_filtered_sensor", 1);

  object_state_.name.push_back("articulation_joint");
  object_state_.position.push_back(0.0);
  object_state_.velocity.push_back(0.0);
  articulation_first_computation_ = true;

  robot_state_.name = {"x_base_joint", "y_base_joint", "pivot_joint"};
  robot_state_.position.resize(robot_state_.name.size());
  robot_state_.velocity.resize(robot_state_.name.size());
  robot_state_.header.frame_id = "world";

  base_twist_ros_.header.frame_id = "world";
}

bool StateObserver::initialize() {
  if (!nh_.getParam("base_alpha", base_alpha_) || base_alpha_ < 0 ||
      base_alpha_ > 1) {
    ROS_ERROR("Failed to parse base_alpha param or invalid.");
    return false;
  }

  if (!simulation_) {
    KDL::Tree object_kinematics;
    if (!kdl_parser::treeFromParam("object_description", object_kinematics)) {
      ROS_ERROR("Failed to create KDL::Tree from 'object_description'");
      return false;
    }

    KDL::Chain chain;
    object_kinematics.getChain("shelf", "handle_link", chain);
    if (chain.getNrOfJoints() > 1) {
      ROS_ERROR(
          "The object has more then one joint. Only one joint supported!");
      return false;
    }
    if (chain.getNrOfJoints() == 0) {
      ROS_ERROR("Failed to parse the object kinematic chain.");
      return false;
    }

    // at start-up door is closed
    KDL::JntArray joint_pos(chain.getNrOfJoints());
    

    // required to calibrate the initial shelf position
    KDL::Frame T_shelf_handle_KDL;
    KDL::ChainFkSolverPos_recursive fk_solver_shelf(chain);
    fk_solver_shelf.JntToCart(joint_pos, T_shelf_handle_KDL);
    tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

    // required to now the origin hinge position
    KDL::Frame T_door_handle_KDL;
    object_kinematics.getChain("axis_link", "handle_link", chain);
    KDL::ChainFkSolverPos_recursive fk_solver_hinge(chain);
    fk_solver_hinge.JntToCart(joint_pos, T_door_handle_KDL);
    tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);
  }

  // get the relative pose of base to reference frame
  KDL::Tree robot_kinematics;
  if (!kdl_parser::treeFromParam("robot_description", robot_kinematics)) {
    ROS_ERROR("Failed to create KDL::Tree from 'robot_description'");
    return false;
  }

  KDL::Chain robot_chain;
  // if (!robot_kinematics.getChain("base_link", "reference_link", robot_chain)) {
  //   ROS_ERROR("Failed to extract chain from base_link to reference_link");
  //   return false;
  // }

  if (!robot_kinematics.getChain("world", "panda_hand", world_to_ee_chain_)) {
    ROS_ERROR("Failed to extract chain from world to panda_hand");
    return false;
  }
  kdl_joints_.resize(world_to_ee_chain_.getNrOfJoints());
  J_world_ee_.resize(world_to_ee_chain_.getNrOfJoints());
  jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(world_to_ee_chain_);

  KDL::Frame T_base_reference_KDL;
  KDL::JntArray robot_joint_pos(robot_chain.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive robot_solver(robot_chain);
  robot_solver.JntToCart(robot_joint_pos, T_base_reference_KDL);
  tf::transformKDLToEigen(T_base_reference_KDL.Inverse(), T_reference_base_);

  ROS_INFO_STREAM("Static transformations summary: "
                  << std::endl
                  << " T_base_reference:\n "
                  << T_reference_base_.inverse().matrix() << std::endl);

  if (!nh_.getParam("wrench_threshold", wrench_threshold_) ||
      wrench_threshold_ < 0) {
    ROS_ERROR("Failed to parse wrench_threshold param or invalid.");
    return false;
  }

  if (!nh_.getParam("wrench_contact_threshold", wrench_contact_threshold_)) {
    ROS_ERROR("Failed to parse wrench_contact_threshold param or invalid.");
    return false;
  }

  wrench_median_filter_ =
      std::make_unique<filters::MultiChannelMedianFilter<double>>();
  if (!wrench_median_filter_->configure(6, "wrench_median_filter", nh_)) {
    ROS_ERROR("Failed to initialize the wrench median filter");
    return false;
  }

  wrench_meas_v_.resize(6, 0.0);
  wrench_medf_.resize(6, 0.0);

  wrench_meas_.setZero(6);
  wrench_filt_.setZero(6);
  wrench_filt_sensor_.setZero(6);

  ROS_INFO("Robot observer correctly initialized.");
  return true;
}

void StateObserver::odom_callback(const nav_msgs::OdometryConstPtr& msg) {
  // Update base pose using Odometry
  base_pose_.x() = msg->pose.pose.position.x;
  base_pose_.y() = msg->pose.pose.position.y;

  // Extract orientation from quaternion to yaw angle
  Eigen::Quaterniond q;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  q.w() = msg->pose.pose.orientation.w;

  // Convert quaternion to yaw (2D) using atan2 to avoid Euler angle issues
  Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  base_pose_.z() = yaw;  // update the yaw angle

  // Update base twist using Odometry
  base_twist_.x() = msg->twist.twist.linear.x;
  base_twist_.y() = msg->twist.twist.linear.y;
  base_twist_.z() = msg->twist.twist.angular.z;

  // Publish updated base pose and twist
  geometry_msgs::PoseStamped base_pose;
  base_pose.header.stamp = ros::Time::now();
  base_pose.header.frame_id = "world";
  base_pose.pose.position.x = base_pose_.x();
  base_pose.pose.position.y = base_pose_.y();
  base_pose.pose.position.z = 0.0;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(base_pose_.z(), Eigen::Vector3d::UnitZ()));
  base_pose.pose.orientation.x = quat.x();
  base_pose.pose.orientation.y = quat.y();
  base_pose.pose.orientation.z = quat.z();
  base_pose.pose.orientation.w = quat.w();
  base_pose_publisher_.publish(base_pose);

  // Update and publish robot state
  robot_state_.header.stamp = msg->header.stamp;
  robot_state_.position[0] = base_pose_.x();
  robot_state_.position[1] = base_pose_.y();
  robot_state_.position[2] = base_pose_.z();
  robot_state_publisher_.publish(robot_state_);

  // Update base twist
  base_twist_ros_.header.stamp = msg->header.stamp;
  base_twist_ros_.twist.linear.x = base_twist_.x();
  base_twist_ros_.twist.linear.y = base_twist_.y();
  base_twist_ros_.twist.angular.z = base_twist_.z();
  base_twist_publisher_.publish(base_twist_ros_);
}

void StateObserver::arm_state_callback(const sensor_msgs::JointStateConstPtr& msg) {
  if (!are_equal((int)(9), (int)msg->name.size(), (int)msg->position.size(),
                 (int)msg->effort.size(), (int)msg->velocity.size())) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Joint state fields have the wrong size." << msg->name.size());
    return;
  }

  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < 9; i++) {
      q_(i) = msg->position[i];
      dq_(i) = msg->velocity[i];
    }
  }

  previous_publishing_time_ = time_;
}

void StateObserver::object_state_callback(
    const sensor_msgs::JointStateConstPtr& msg) {
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    object_state_.header.stamp = msg->header.stamp;
    object_state_.position[0] = msg->position[0];
    object_state_.velocity[0] = msg->velocity[0];
  }
  object_state_publisher_.publish(object_state_);
}

void StateObserver::object_pose_callback(
    const nav_msgs::OdometryConstPtr& msg) {
  if (simulation_) return;
  tf::poseMsgToEigen(msg->pose.pose, T_world_handle_);
  if (articulation_first_computation_) {
    T_world_shelf_ = T_world_handle_ * T_handle0_shelf_;
    T_hinge_world_ = (T_world_handle_ * T_handle0_hinge_).inverse();
    T_hinge_handle_init_ = T_hinge_world_ * T_world_handle_;

    start_relative_angle_ = std::atan2(T_hinge_handle_init_.translation().x(),
                                       T_hinge_handle_init_.translation().y());
    articulation_first_computation_ = false;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped T_world_shelf_ros;
    tf::transformEigenToMsg(T_world_shelf_, T_world_shelf_ros.transform);
    T_world_shelf_ros.header.stamp = ros::Time::now();
    T_world_shelf_ros.header.frame_id = "world";
    T_world_shelf_ros.child_frame_id = "shelf";
    static_broadcaster.sendTransform(T_world_shelf_ros);
    ROS_INFO_STREAM(
        "T_world_shelf is:"
        << std::endl
        << "translation: " << T_world_shelf_.translation().transpose()
        << std::endl
        << "rotation: "
        << T_world_shelf_.rotation().eulerAngles(0, 1, 2).transpose());
    ROS_INFO_STREAM("Published initial transform from world to shelf frame.");
    return;
  }

  T_hinge_handle_ = T_hinge_world_ * T_world_handle_;
  current_relative_angle_ = std::atan2(T_hinge_handle_.translation().x(),
                                       T_hinge_handle_.translation().y());

  double theta_new = current_relative_angle_ - start_relative_angle_;

  double velocity_norm =
      std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  double theta_dot = velocity_norm / T_hinge_handle_.translation().head<2>().norm();

  {
    std::unique_lock<std::mutex> lock(state_mutex_);  
    object_state_.header.stamp = msg->header.stamp;
    object_state_.position[0] = theta_new;
    object_state_.velocity[0] = theta_dot;
  }
  object_state_publisher_.publish(object_state_);
}

void StateObserver::filter_wrench() {
  // median filter to get rid of spikes
  for (int i = 0; i < 6; i++) {
    wrench_meas_v_[i] = wrench_meas_[i];
  }
  wrench_median_filter_->update(wrench_meas_v_, wrench_medf_);

  for (int i = 0; i < 6; i++) {
    wrench_filt_[i] = wrench_medf_[i]; //wrench_lp_filters_[i].apply(wrench_medf_[i]);
  }

  // thresholding to remove uncompensated bias/payload

  if (wrench_filt_.norm() < wrench_threshold_) {
    wrench_filt_.setZero();
  } else {
    wrench_filt_ -= wrench_threshold_ * wrench_filt_ / wrench_filt_.norm();
  }
}

void StateObserver::wrench_callback(
    const geometry_msgs::WrenchStampedConstPtr& msg) {
  // wrench is in the end effector frame. Need to convert to world frame
  Eigen::Affine3d T_world_sensor;
  geometry_msgs::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform("world", msg->header.frame_id,
                                           ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  tf::transformMsgToEigen(transform.transform, T_world_sensor);

  // update kdl joints with the latest measurements
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    kdl_joints_(0) = base_pose_.x();
    kdl_joints_(1) = base_pose_.y();
    kdl_joints_(2) = base_pose_.z();
    for (int i = 0; i < 7; i++) {
      kdl_joints_(3 + i) = q_(i);
    }
  }
  jacobian_solver_->JntToJac(kdl_joints_, J_world_ee_);

  // filter wrench measurements
  const static double alpha = 0.1;
  wrench_meas_(0) = msg->wrench.force.x;
  wrench_meas_(1) = msg->wrench.force.y;
  wrench_meas_(2) = msg->wrench.force.z;
  wrench_meas_(3) = msg->wrench.torque.x;
  wrench_meas_(4) = msg->wrench.torque.y;
  wrench_meas_(5) = msg->wrench.torque.z;

  // rotate to world frame to filter wrench in a inertial frame
  wrench_meas_.head<3>() = T_world_sensor.rotation() * wrench_meas_.head<3>();
  wrench_meas_.tail<3>() = T_world_sensor.rotation() * wrench_meas_.tail<3>();


  // filter wrench to eliminate spikes and small bias
  filter_wrench();

  // publish wrench filtered
  wrench_filt_sensor_.head<3>() = T_world_sensor.rotation().transpose() * wrench_filt_.head<3>();
  wrench_filt_sensor_.tail<3>() = T_world_sensor.rotation().transpose() * wrench_filt_.tail<3>();
  wrench_filt_ros_.wrench.force.x = wrench_filt_sensor_(0);
  wrench_filt_ros_.wrench.force.y = wrench_filt_sensor_(1);
  wrench_filt_ros_.wrench.force.z = wrench_filt_sensor_(2);
  wrench_filt_ros_.wrench.torque.x = wrench_filt_sensor_(3);
  wrench_filt_ros_.wrench.torque.y = wrench_filt_sensor_(4);
  wrench_filt_ros_.wrench.torque.z = wrench_filt_sensor_(5);
  wrench_filt_ros_.header = msg->header;
  wrench_filt_publisher_.publish(wrench_filt_ros_);
  
  
  // clang-format off
  J_world_ee_.data.topLeftCorner<3, 3>() << std::cos(base_pose_.z()), -std::sin(base_pose_.z()), 0.0,
                                            std::sin(base_pose_.z()), std::cos(base_pose_.z()), 0.0,
                                            0.0, 0.0, 1.0;
  // clang-format on
  ext_tau_ = J_world_ee_.data.transpose() * wrench_filt_;

  // sanitize torque
  {
    std::unique_lock<std::mutex> lock(state_mutex_);  
    for (int i = 0; i < ext_tau_.size(); i++) {
      ext_tau_(i) = std::abs(ext_tau_(i)) > 1e3 ? 0.0 : ext_tau_(i);
    }
    // detect contact from wrench using small threshold
    contact_state_ = wrench_meas_.norm() > wrench_contact_threshold_;
  }
}

void StateObserver::publish_state() {
  std::unique_lock<std::mutex> lock(state_mutex_);
  manipulation::conversions::toMsg(
      ros::Time::now().toSec(), base_pose_, base_twist_, ext_tau_.head<3>(), q_, dq_,
      ext_tau_.tail<9>(), object_state_.position[0], object_state_.velocity[0],
      contact_state_, state_ros_);

  state_publisher_.publish(state_ros_);
}
}  // namespace manipulation_royalpanda
