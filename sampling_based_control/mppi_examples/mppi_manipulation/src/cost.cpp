/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/cost.h"
#include <ros/package.h>
#include "mppi_manipulation/dimensions.h"
#include <std_msgs/Float32.h>
#include <limits>
#include <fcl/fcl.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

using namespace manipulation;

// Static variable to store DBB points loaded from the file
static std::vector<std::tuple<Eigen::Vector3f, double>> static_dbb_points;
bool first_search = true;
Eigen::Vector3f last_acting_point;
double distance_;

PandaCost::PandaCost(const CostParams& params) : params_(params), dbb_distance_(std::numeric_limits<float>::max()), acting_point_(Eigen::Vector3f::Zero()) {
  robot_model_.init_from_xml(params_.robot_description);
  object_model_.init_from_xml(params_.object_description);

  dbb_distance_sub_ = nh_.subscribe("/dbb_distance", 10, &PandaCost::dbbDistanceCallback, this);
  acting_point_sub_ = nh_.subscribe("/acting_point", 10, &PandaCost::actingPointCallback, this);
}

void PandaCost::dbbDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  dbb_distance_ = msg->data;
}
void PandaCost::actingPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    acting_point_ = Eigen::Vector3f(msg->x, msg->y, msg->z);  // Update acting_point_ with received data
}

void PandaCost::createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model) {
    // Base link SSV
    SSV AB, CD;

    // Get the base link's position from the rollout state
    Eigen::Vector3d base_pos = robot_model.get_pose("base_link").translation;

    AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    AB.radius = 0.3;
    ssvs.push_back(AB);

    // Use capsule collision model for DBB_mesh
    CD.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    CD.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    CD.radius = 0.45;
    ssvs.push_back(CD);

    // End effector SSV
    SSV I;
    I.point1 = robot_model.get_pose("panda_link7").translation;
    I.point2 = robot_model.get_pose("panda_link8").translation;
    I.radius = 0.1;
    ssvs.push_back(I);
}

// Create FCL collision objects
void PandaCost::createFCLObjects(std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects, const std::vector<SSV>& ssvs) {
    fcl_objects.clear();

    for (const auto& ssv : ssvs) {
        fcl::Vector3d fcl_point1(ssv.point1.x(), ssv.point1.y(), ssv.point1.z());
        fcl::Vector3d fcl_point2(ssv.point2.x(), ssv.point2.y(), ssv.point2.z());

        fcl::Transform3d transform;
        transform.setIdentity();

        // Calculate the midpoint and orientation of the capsule
        fcl::Vector3d capsule_midpoint = (fcl_point1 + fcl_point2) * 0.5;
        fcl::Vector3d capsule_axis = (fcl_point2 - fcl_point1).normalized();
        
        // Create a transformation to place the capsule correctly
        transform.translation() = capsule_midpoint;
        transform.linear().col(2) = capsule_axis;

        auto capsule = std::make_shared<fcl::CollisionObjectd>(
            std::make_shared<fcl::Capsuled>(ssv.radius, (fcl_point1 - fcl_point2).norm()), transform
        );
        fcl_objects.push_back(capsule);
    }
}

// Find closest point on EE between DBB
double PandaCost::EEToDBB(const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects) {
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    
    double distance = fcl::distance(fcl_objects[1].get(), fcl_objects[2].get(), request, result);

    return distance;
}

mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                     mppi::input_t& u,
                                     const mppi::reference_t& ref,
                                     const double t) {
    double cost = 0;
    int mode = 0;
    double reach;
    double base_regularization_weight = 0.5;  // Lower weight to encourage base movement
    double arm_regularization_weight = 1.0;   // Higher weight to discourage arm movement

    robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
    object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));

    // Self-collision avoidance using acting point
    std::vector<SSV> ssvs;
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> fcl_objects;
    createSSVsForRollout(ssvs, robot_model_);
    createFCLObjects(fcl_objects, ssvs);
    double distance = EEToDBB(fcl_objects);

    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);

    if (dbb_distance_ > 0) {
        
        // End-effector reaching cost
        cost += (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
        cost += (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

        // Joint limits cost
        for (size_t i = 0; i < 10; i++) {
            if (x(i) < params_.lower_joint_limits[i])
                cost += params_.Q_joint_limit + params_.Q_joint_limit_slope * std::pow(params_.lower_joint_limits[i] - x(i), 2);

            if (x(i) > params_.upper_joint_limits[i])
                cost += params_.Q_joint_limit + params_.Q_joint_limit_slope * std::pow(x(i) - params_.upper_joint_limits[i], 2);
        }

        // Arm reach cost
        robot_model_.get_offset(params_.arm_base_frame, params_.tracked_frame,
                                distance_vector_);
        reach = distance_vector_.head<2>().norm();
        if (reach > params_.max_reach) {
            cost += params_.Q_reach +
                    params_.Q_reachs * (std::pow(reach - params_.max_reach, 2));
        }

        if (distance_vector_.norm() < params_.min_dist) {
            cost += params_.Q_reach +
                    params_.Q_reachs * (std::pow(reach - params_.min_dist, 2));
        }
    }

    // End-effector inside DBB
    if (dbb_distance_ <= 0) { 

        // Apply regularization with different weights for the base and arm segments
        // cost += base_regularization_weight * x.segment<3>(0).norm() + 
        //         arm_regularization_weight * x.segment<7>(3).norm();

        cost += (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
        cost += (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

        if (acting_point_ != Eigen::Vector3f::Zero()) {
            double x = acting_point_.x();
            double y = acting_point_.y();
            
            // Calculate the desired velocity from the acting point
            double distance_to_origin = std::sqrt(x * x + y * y);
            double target_angle = std::atan2(y, x);
            
            Eigen::Vector2d acting_point_velocity = Eigen::Vector2d::Zero();
            acting_point_velocity.x() = -distance_to_origin;  // Linear velocity
            acting_point_velocity.y() = target_angle;         // Angular velocity

            // Extract base control input
            Eigen::Vector2d base_speed = Eigen::Vector2d::Zero();
            base_speed.x() = u(0);
            base_speed.y() = u(2);

            u(0) = acting_point_velocity.x();
            u(2) = acting_point_velocity.y();

            // Normalize both vectors for alignment comparison
            Eigen::Vector2d normalized_target_velocity = acting_point_velocity.normalized();
            Eigen::Vector2d normalized_base_velocity = base_speed.normalized();

            // Calculate dot product for directional alignment
            double direction_alignment = normalized_target_velocity.dot(normalized_base_velocity);

            // Magnitude difference for speed comparison
            double magnitude_difference = std::abs(acting_point_velocity.norm() - base_speed.norm());

            // Add costs based on alignment and magnitude
            cost += (1 - direction_alignment) + magnitude_difference;
        }
    }

    return cost;
}

// Example reference publisher

// rostopic pub /end_effector_pose_desired geometry_msgs/PoseStamped "header:
//   seq: 0
//   stamp:
//     secs: 0
//     nsecs: 0
//   frame_id: 'world'
// pose:
//   position:
//     x: 0.259
//     y: 0.0
//     z: 0.5
//   orientation:
//     x: 0.6995708346366882
//     y: 0.7145636677742004
//     z: -1.6931199553482656e-08
//     w: 1.2759413969831712e-08"
