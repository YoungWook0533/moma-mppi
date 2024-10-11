#pragma once

#include <math.h>
#include <mppi/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>  
#include <Eigen/Dense>  // Include Eigen for vector calculations
#include <vector>       // For using std::vector

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
                            const mppi::input_t& u,
                            const mppi::reference_t& ref,
                            const double t) override;

  // Create SSVs for rollout and calculate minimum distance
  void createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model);

  double calculateDistance(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,const Eigen::Vector3d& Q1, const Eigen::Vector3d& Q2);

  double calculateMinDistance(const std::vector<SSV>& ssvs);
};

}  // namespace manipulation
