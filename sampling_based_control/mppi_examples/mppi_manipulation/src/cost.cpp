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

    AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    AB.radius = 0.3;
    ssvs.push_back(AB);

    CD1.point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
    CD1.point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
    CD1.radius = 0.09;
    ssvs.push_back(CD1);

    CD2.point1 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 0.481);
    CD2.point2 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 1.3);
    CD2.radius = 0.09;
    ssvs.push_back(CD2);

    // Manipulator SSVs
    SSV EF, FG, GH, I;

    // Use the robot model to get the positions of the manipulator links based on rollout joint states
    Eigen::Vector3d panda_link0 = robot_model.get_pose("panda_link0").translation;
    Eigen::Vector3d panda_link2 = robot_model.get_pose("panda_link2").translation;

    EF.point1 = panda_link0;
    EF.point2 = panda_link2;
    EF.radius = 0.07;
    ssvs.push_back(EF);

    FG.point1 = panda_link2;
    FG.point2 = robot_model.get_pose("panda_link3").translation;
    FG.radius = 0.07;
    ssvs.push_back(FG);

    GH.point1 = robot_model.get_pose("panda_link_4_1").translation;
    GH.point2 = Eigen::Vector3d(0, 0.025, 0) + robot_model.get_pose("panda_link5").translation;
    GH.radius = 0.09;
    ssvs.push_back(GH);

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

// Calculate the distance between two FCL objects
double PandaCost::calculateDistance(const std::shared_ptr<fcl::CollisionObjectd>& obj1, const std::shared_ptr<fcl::CollisionObjectd>& obj2) {
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;

    // Perform distance calculation
    double distance = fcl::distance(obj1.get(), obj2.get(), request, result);

    return distance;
}


double PandaCost::calculateMinDistance(const std::vector<SSV>& ssvs, std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects) {
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

    for (size_t i = 0; i < fcl_objects.size(); ++i) {
        for (size_t j = i + 1; j < fcl_objects.size(); ++j) {
            bool skip = false;
            for (const auto& pair : ignorePairs) {
                if ((i == pair.first && j == pair.second) || (i == pair.second && j == pair.first)) {
                    skip = true;
                    break;
                }
            }
            if (skip) continue;

            // Calculate the distance
            double distance = calculateDistance(fcl_objects[i], fcl_objects[j]);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    }
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
  std::vector<std::shared_ptr<fcl::CollisionObjectd>> fcl_objects;
  createSSVsForRollout(ssvs, robot_model_);  // Calculate link positions based on rollout joint states
  createFCLObjects(fcl_objects, ssvs);
  double min_distance = calculateMinDistance(ssvs, fcl_objects);

  // if(min_distance < 0.05)
  // {
  //   cost += 1000000 * std::pow(std::max(0.0, params_.collision_threshold - min_distance), 2);
  // }

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


// /*!
//  * @file     pendulum_cart_cost.cpp
//  * @author   Giuseppe Rizzi
//  * @date     10.06.2020
//  * @version  1.0
//  * @brief    description
//  */

// #include "mppi_manipulation/cost.h"
// #include <ros/package.h>
// #include "mppi_manipulation/dimensions.h"
// #include <std_msgs/Float32.h>
// #include <limits>
// #include <fcl/fcl.h>

// using namespace manipulation;

// PandaCost::PandaCost(const CostParams& params) : params_(params) {
//   robot_model_.init_from_xml(params_.robot_description);
//   object_model_.init_from_xml(params_.object_description);
// }

// void PandaCost::createSSVsForRollout(std::vector<SSV>& ssvs, const mppi_pinocchio::RobotModel& robot_model) {
//     // Base link SSVs
//     // SSV AB, CD1, CD2;

    
//     // fcl::CollisionObjectd link2(std::make_shared<fcl::Capsuled>(0.05, 0.4), fcl::Transform3d());

//     // Get the base link's position from the rollout state
//     Eigen::Vector3d base_pos = robot_model.get_pose("base_link").translation;
//     fcl::Transform3d ab;
//     ab.translation() = base_pos;
//     fcl::CollisionObjectd AB(std::make_shared<fcl::Capsuled>(0.3, 0.5552), ab);

//     Eigen::Vector3d CD1_point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
//     Eigen::Vector3d CD1_point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
//     Eigen::Vector3d midpoint = (CD1_point1 + CD1_point2) / 2.0;
//     Eigen::Vector3d direction = CD1_point2 - CD1_point1;
//     double length = direction.norm();
//     Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction.normalized());
//     fcl::Transform3d cd1;
//     cd1.translation() = midpoint;
//     cd1.linear() = orientation.toRotationMatrix();
//     fcl::CollisionObjectd CD1(std::make_shared<fcl::Capsuled>(0.09, length), cd1);

//     Eigen::Vector3d CD2_point1 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 0.481);
//     Eigen::Vector3d CD2_point2 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 1.3);
//     midpoint = (CD2_point1 + CD2_point2) / 2.0;
//     direction = CD2_point2 - CD2_point1;
//     length = direction.norm();
//     orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction.normalized());
//     fcl::Transform3d cd2;
//     cd2.translation() = midpoint;
//     cd2.linear() = orientation.toRotationMatrix();
//     fcl::CollisionObjectd CD2(std::make_shared<fcl::Capsuled>(0.09, length), cd2);

//     // AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
//     // AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
//     // AB.radius = 0.3;
//     // ssvs.push_back(AB);

//     // CD1.point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
//     // CD1.point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
//     // CD1.radius = 0.09;
//     // ssvs.push_back(CD1);

//     // CD2.point1 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 0.481);
//     // CD2.point2 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 1.3);
//     // CD2.radius = 0.09;
//     // ssvs.push_back(CD2);

//     // Manipulator SSVs
//     // SSV EF, FG, GH, I;

//     // Use the robot model to get the positions of the manipulator links based on rollout joint states
//     Eigen::Vector3d panda_link0 = robot_model.get_pose("panda_link0").translation;
//     Eigen::Vector3d panda_link2 = robot_model.get_pose("panda_link2").translation;

//     fcl::Transform3d ef;
//     ef.translation() = panda_link0;
//     fcl::CollisionObjectd EF(std::make_shared<fcl::Capsuled>(0.07, 0.1), ef);
    
//     // EF.point1 = panda_link0;
//     // EF.point2 = panda_link2;
//     // EF.radius = 0.07;
//     // ssvs.push_back(EF);

//     // FG.point1 = panda_link2;
//     // FG.point2 = robot_model.get_pose("panda_link3").translation;
//     // FG.radius = 0.07;
//     // ssvs.push_back(FG);

//     // GH.point1 = robot_model.get_pose("panda_link_4_1").translation;
//     // GH.point2 = Eigen::Vector3d(0, 0.025, 0) + robot_model.get_pose("panda_link5").translation;
//     // GH.radius = 0.09;
//     // ssvs.push_back(GH);

//     // I.point1 = robot_model.get_pose("panda_link7").translation;
//     // I.point2 = robot_model.get_pose("panda_link8").translation;
//     // I.radius = 0.1;
//     // ssvs.push_back(I);
// }

// double PandaCost::calculateDistance(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
//                          const Eigen::Vector3d& Q1, const Eigen::Vector3d& Q2) {
//     Eigen::Vector3d u = P2 - P1;
//     Eigen::Vector3d v = Q2 - Q1;
//     Eigen::Vector3d w = P1 - Q1;

//     double a = u.dot(u);
//     double b = u.dot(v);
//     double c = v.dot(v);
//     double d = u.dot(w);
//     double e = v.dot(w);

//     double denominator = a * c - b * b;

//     double s, t;
//     if (denominator < 1e-8) {  // Lines are almost parallel
//         s = 0.0;
//         t = (b > c ? d / b : e / c);
//     } else {
//         s = (b * e - c * d) / denominator;
//         t = (a * e - b * d) / denominator;
//     }

//     // Clamp s and t to the range [0, 1]
//     s = std::max(0.0, std::min(1.0, s));
//     t = std::max(0.0, std::min(1.0, t));

//     // Closest points on the two segments
//     Eigen::Vector3d P_closest = P1 + s * u;
//     Eigen::Vector3d Q_closest = Q1 + t * v;

//     double distance = (P_closest - Q_closest).norm();

//     return distance;
// }


// double PandaCost::calculateMinDistance(const std::vector<SSV>& ssvs) {
//     double min_distance = std::numeric_limits<double>::max();

//     std::vector<std::pair<int, int>> ignorePairs = {
//         {0, 1},  // AB and CD1
//         {0, 2},  // AB and CD2
//         {1, 2},  // AB and CD2
//         {0, 3},  // AB and EF
//         {3, 4},  // EF and FG
//         {4, 5},  // FG and GH
//         {5, 6}   // GH and I
//     };

//     for (size_t i = 0; i < ssvs.size(); ++i) {
//         for (size_t j = i + 1; j < ssvs.size(); ++j) {
//             bool skip = false;
//             for (const auto& pair : ignorePairs) {
//                 if ((i == pair.first && j == pair.second) || (i == pair.second && j == pair.first)) {
//                     skip = true;
//                     break;
//                 }
//             }
//             if (skip) continue;

//             // Calculate distance and subtract radii
//             double distance = calculateDistance(ssvs[i].point1, ssvs[i].point2, ssvs[j].point1, ssvs[j].point2);
//             double link_distance = distance - (ssvs[i].radius + ssvs[j].radius);
//             if (!std::isnan(link_distance) && link_distance < min_distance) {
//                 min_distance = link_distance;
//             }
//         }
//     }
//     // ROS_INFO_STREAM("min_distance = " << min_distance);
//     return min_distance;
// }


// mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
//                                      const mppi::input_t& u,
//                                      const mppi::reference_t& ref,
//                                      const double t) {
//   double cost = 0.0;
  
//   // int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);
//   int mode = 0;

//   robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
//   object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));
  
//   // regularization cost
//   cost += params_.Qreg *
//           x.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM).norm();
  
//   // end effector reaching cost
//   if (mode == 0) {
//     Eigen::Vector3d ref_t = ref.head<3>();
//     Eigen::Quaterniond ref_q(ref.segment<4>(3));
//     robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
//     cost +=
//         (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
//     cost +=
//         (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

//     if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) cost += params_.Qc;
//   }

//   // handle reaching cost
//   else if (mode == 1) {
//     error_ = mppi_pinocchio::diff(
//         robot_model_.get_pose(params_.tracked_frame),
//         object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
//     cost +=
//         (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
//     cost +=
//         (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

//     // contact cost
//     if (x(2*BASE_ARM_GRIPPER_DIM + 2*OBJECT_DIMENSION) > 0) {
//       cost += params_.Qc;
//     }
//   }

//   // object displacement cost
//   else if (mode == 2) {
//     error_ = mppi_pinocchio::diff(
//         robot_model_.get_pose(params_.tracked_frame),
//         object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
//     cost +=
//         (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt2;
//     cost +=
//         (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr2;

//     double object_error =
//         x(2 * BASE_ARM_GRIPPER_DIM) -
//         ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);

//     cost += object_error * object_error * params_.Q_obj;
//   }
  
//   // power cost
//   // cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power); 
  
//   // self collision cost
// //   robot_model_.get_offset(params_.collision_link_0, params_.collision_link_1,
// //                           collision_vector_);
// //   cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);
  
//   // TODO(giuseppe) hard coded for now to match the collision pairs of the safety filter
// //   robot_model_.get_offset("panda_link0", "panda_link7", collision_vector_);
// //   cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);

//   // SSV-based self-collision cost <TODO : calculate minimum link distance for simulated joint states, not current joint states>
//   std::vector<SSV> ssvs;
//   createSSVsForRollout(ssvs, robot_model_);  // Calculate link positions based on rollout joint states
//   double min_distance = calculateMinDistance(ssvs);

//   if(min_distance < 0.05)
//   {
//     cost += 1000000 * std::pow(std::max(0.0, params_.collision_threshold - min_distance), 2);
//   }

//   // cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - min_distance), 2);

//   // arm reach cost
//   double reach;
//   robot_model_.get_offset(params_.arm_base_frame, params_.tracked_frame,
//                           distance_vector_);
//   reach = distance_vector_.head<2>().norm();
//   if (reach > params_.max_reach) {
//     cost += params_.Q_reach +
//             params_.Q_reachs * (std::pow(reach - params_.max_reach, 2));
//   }

//   if (distance_vector_.norm() < params_.min_dist) {
//     cost += params_.Q_reach +
//             params_.Q_reachs * (std::pow(reach - params_.min_dist, 2));
//   }
  
//   // joint limits cost
//   for (size_t i = 0; i < 10; i++) {
//     if (x(i) < params_.lower_joint_limits[i])
//       cost += params_.Q_joint_limit +
//               params_.Q_joint_limit_slope *
//                   std::pow(params_.lower_joint_limits[i] - x(i), 2);

//     if (x(i) > params_.upper_joint_limits[i])
//       cost += params_.Q_joint_limit +
//               params_.Q_joint_limit_slope *
//                   std::pow(x(i) - params_.upper_joint_limits[i], 2);
//   }

//   return cost;
// }
