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


using namespace manipulation;

PandaCost::PandaCost(const CostParams& params) : params_(params) {
  robot_model_.init_from_xml(params_.robot_description);
  object_model_.init_from_xml(params_.object_description);
}

void PandaCost::createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model) {
    // Base link SSVs
    SSV AB, CD1, CD2;

    // Get the base link's position from the rollout state
    Eigen::Vector3d base_pos = robot_model.get_pose("base_link").translation;

    // Segment AB (Base link points A and B)
    AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    AB.radius = 0.3;
    ssvs.push_back(AB);

    // Segment CD1 (Base link points C1 and D1)
    CD1.point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
    CD1.point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
    CD1.radius = 0.09;
    ssvs.push_back(CD1);

    // Segment CD2 (Base link points C2 and D2)
    CD2.point1 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 0.481);
    CD2.point2 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 1.3);
    CD2.radius = 0.09;
    ssvs.push_back(CD2);

    // Manipulator SSVs
    SSV EF, FG, GH, I;

    // Use the robot model to get the positions of the manipulator links based on rollout joint states
    Eigen::Vector3d panda_link0 = robot_model.get_pose("panda_link0").translation;
    Eigen::Vector3d panda_link2 = robot_model.get_pose("panda_link2").translation;

    // Segment EF (Link 0 to Link 2)
    EF.point1 = panda_link0;
    EF.point2 = panda_link2;
    EF.radius = 0.07;  // Adjust the radius for the manipulator links
    ssvs.push_back(EF);

    // Segment FG (Link 2 to Link 3)
    FG.point1 = panda_link2;
    FG.point2 = robot_model.get_pose("panda_link3").translation;
    FG.radius = 0.07;
    ssvs.push_back(FG);

    // Segment GH (Link 4 to Link 5)
    GH.point1 = robot_model.get_pose("panda_link_4_1").translation;
    GH.point2 = Eigen::Vector3d(0, 0.025, 0) + robot_model.get_pose("panda_link5").translation;
    GH.radius = 0.09;
    ssvs.push_back(GH);

    // Segment I (Link 7 to Link 8)
    I.point1 = robot_model.get_pose("panda_link7").translation;
    I.point2 = robot_model.get_pose("panda_link8").translation;
    I.radius = 0.1;
    ssvs.push_back(I);
}

double PandaCost::calculateDistance(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                         const Eigen::Vector3d& Q1, const Eigen::Vector3d& Q2) {
    Eigen::Vector3d u = P2 - P1;
    Eigen::Vector3d v = Q2 - Q1;
    Eigen::Vector3d w = P1 - Q1;    // Vector between the starting points of the two segments

    double a = u.dot(u);  // Squared length of segment 1
    double b = u.dot(v);  // Projection of segment 1 on segment 2
    double c = v.dot(v);  // Squared length of segment 2
    double d = u.dot(w);  // Projection of w on segment 1
    double e = v.dot(w);  // Projection of w on segment 2

    double denominator = a * c - b * b;  // Always non-negative

    double s, t;
    if (denominator < 1e-8) {  // Lines are almost parallel
        s = 0.0;
        t = (b > c ? d / b : e / c);
    } else {
        s = (b * e - c * d) / denominator;
        t = (a * e - b * d) / denominator;
    }

    // Clamp s and t to the range [0, 1]
    s = std::max(0.0, std::min(1.0, s));
    t = std::max(0.0, std::min(1.0, t));

    // Closest points on the two segments
    Eigen::Vector3d P_closest = P1 + s * u;
    Eigen::Vector3d Q_closest = Q1 + t * v;

    // Compute the distance between the closest points
    double distance = (P_closest - Q_closest).norm();

    return distance;
}


double PandaCost::calculateMinDistance(const std::vector<SSV>& ssvs) {
    double min_distance = std::numeric_limits<double>::max();

    std::vector<std::pair<int, int>> ignorePairs = {
        {0, 1},  // AB and CD1
        {0, 2},  // AB and CD2
        {1, 2},  // AB and CD2
        {0, 3},  // AB and EF
        {3, 4},  // EF and FG
        {4, 5},  // FG and GH
        {5, 6}   // GH and I
    };

    for (size_t i = 0; i < ssvs.size(); ++i) {
        for (size_t j = i + 1; j < ssvs.size(); ++j) {
            bool skip = false;
            for (const auto& pair : ignorePairs) {
                if ((i == pair.first && j == pair.second) || (i == pair.second && j == pair.first)) {
                    skip = true;
                    break;
                }
            }
            if (skip) continue;

            // Calculate distance and subtract radii
            double distance = calculateDistance(ssvs[i].point1, ssvs[i].point2, ssvs[j].point1, ssvs[j].point2);
            double link_distance = distance - (ssvs[i].radius + ssvs[j].radius);
            if (!std::isnan(link_distance) && link_distance < min_distance) {
                min_distance = link_distance;
            }
        }
    }
    // ROS_INFO_STREAM("min_distance = " << min_distance);
    return min_distance;
}


mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                     const mppi::input_t& u,
                                     const mppi::reference_t& ref,
                                     const double t) {
  double cost = 0.0;
  
  // int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);
  int mode = 0;

  robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));
  
  // regularization cost
  cost += params_.Qreg *
          x.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM).norm();
  
  // end effector reaching cost
  if (mode == 0) {
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) cost += params_.Qc;
  }

  // handle reaching cost
  else if (mode == 1) {
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(params_.tracked_frame),
        object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

    // contact cost
    if (x(2*BASE_ARM_GRIPPER_DIM + 2*OBJECT_DIMENSION) > 0) {
      cost += params_.Qc;
    }
  }

  // object displacement cost
  else if (mode == 2) {
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(params_.tracked_frame),
        object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt2;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr2;

    double object_error =
        x(2 * BASE_ARM_GRIPPER_DIM) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);

    cost += object_error * object_error * params_.Q_obj;
  }
  
  // power cost
  // cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power); 
  
  // self collision cost
//   robot_model_.get_offset(params_.collision_link_0, params_.collision_link_1,
//                           collision_vector_);
//   cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);
  
  // TODO(giuseppe) hard coded for now to match the collision pairs of the safety filter
//   robot_model_.get_offset("panda_link0", "panda_link7", collision_vector_);
//   cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);

  // SSV-based self-collision cost <TODO : calculate minimum link distance for simulated joint states, not current joint states>
  std::vector<SSV> ssvs;
  createSSVsForRollout(ssvs, robot_model_);  // Calculate link positions based on rollout joint states
  double min_distance = calculateMinDistance(ssvs);

  if(min_distance < 0.05)
  {
    cost += 1000000 * std::pow(std::max(0.0, params_.collision_threshold - min_distance), 2);
  }

  // cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - min_distance), 2);

  // arm reach cost
  double reach;
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
  
  // joint limits cost
  for (size_t i = 0; i < 10; i++) {
    if (x(i) < params_.lower_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(params_.lower_joint_limits[i] - x(i), 2);

    if (x(i) > params_.upper_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(x(i) - params_.upper_joint_limits[i], 2);
  }

  return cost;
}
