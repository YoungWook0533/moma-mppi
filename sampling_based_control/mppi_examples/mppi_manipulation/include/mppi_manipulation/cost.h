#pragma once

#include <math.h>
#include <mppi/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>  
#include <Eigen/Dense>  
#include <vector>       
#include <fcl/fcl.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Point.h>
#include "mppi_manipulation/params/cost_params.h"

namespace manipulation {

// Define a structure for SSV (Sphere Swept Volume)
struct SSV {
    Eigen::Vector3d point1;  // Start point of the line segment
    Eigen::Vector3d point2;  // End point of the line segment
    double radius;           // Radius of the sphere
};

class PandaCost : public mppi::Cost {
 public:
  PandaCost() : PandaCost(CostParams()){};
  PandaCost(const CostParams& param);
  ~PandaCost() = default;

  // Debug only
  inline const mppi_pinocchio::RobotModel& robot() const {
    return robot_model_;
  }
  inline const mppi_pinocchio::RobotModel& object() const {
    return object_model_;
  }

 private:
  CostParams params_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  int frame_id_;
  int arm_base_frame_id_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Vector3d distance_vector_;
  Eigen::Vector3d collision_vector_;

  ros::NodeHandle nh_;  // Add NodeHandle for subscribing to topics
  ros::Subscriber dbb_distance_sub_;  // Declare the subscriber here
  ros::Subscriber acting_point_sub_; // Add subscriber for acting_point
  Eigen::Vector3f acting_point_ = Eigen::Vector3f::Zero(); 

  std::vector<std::tuple<Eigen::Vector3f, double>> dbb_points_; // Add member variable for the DBB points
  geometry_msgs::Twist calculated_velocity_; // Add member variable for storing the calculated velocity
  double dbb_distance_;
  bool first_search;
  bool first_entery = true;
  Eigen::Vector3f last_acting_point;
  double distance_;
  
 public:
  mppi::cost_ptr create() override {
    return std::make_shared<PandaCost>(params_);
  }
  mppi::cost_ptr clone() const override {
    return std::make_shared<PandaCost>(*this);
  }

  void set_linear_weight(const double k) { params_.Qt = k; }
  
  void set_angular_weight(const double k) { params_.Qr = k; }
  
  void set_obstacle_radius(const double r) { params_.ro = r; }

  mppi::cost_t compute_cost(const mppi::observation_t& x,
                            mppi::input_t& u,
                            const mppi::reference_t& ref,
                            const double t) override;

  // Create SSVs for rollout and calculate minimum distance
  // void createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model);
  // void createFCLObjects(std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects, const std::vector<SSV>& ssvs);
  // double calculateDistance(const std::shared_ptr<fcl::CollisionObjectd>& obj1, const std::shared_ptr<fcl::CollisionObjectd>& obj2);
  // double calculateMinDistance(const std::vector<SSV>& ssvs, std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects);

  void dbbDistanceCallback(const std_msgs::Float32::ConstPtr& msg);

  void actingPointCallback(const geometry_msgs::Point::ConstPtr& msg);

  void createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model);

  void createFCLObjects(std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects, const std::vector<SSV>& ssvs);

  std::shared_ptr<fcl::CollisionObjectd> createCollisionObjectFromMesh(const std::string& obj_file_path);

  double EEToDBB(const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects);

  Eigen::Vector3f EEToBase(const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects);

  Eigen::Vector3f findActingPoint(const std::vector<std::tuple<Eigen::Vector3f, double>>& dbb_points, const Eigen::Vector3f& ee_position);
  
  std::vector<std::tuple<Eigen::Vector3f, double>> loadAndSortDBBPoints(const std::string& filename);
  
};

}  // namespace manipulation
