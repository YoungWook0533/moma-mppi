#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <fcl/fcl.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Float32.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <geometry_msgs/Point.h>

// Define a structure for SSV (Sphere Swept Volume)
struct SSV {
    Eigen::Vector3d point1;  // Start point of the line segment
    Eigen::Vector3d point2;  // End point of the line segment
    double radius;           // Radius of the sphere
};

// Get link positions
Eigen::Vector3d getLinkPosition(const tf2_ros::Buffer& tf_buffer, const std::string& link_name) {
    try {
        geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("base_link", link_name, ros::Time(0));
        return tf2::transformToEigen(transformStamped.transform).translation();
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return Eigen::Vector3d::Zero();  // Return default vector if not found
    }
}

// Create the SSVs
void createSSVs(std::vector<SSV>& ssvs, const tf2_ros::Buffer& tf_buffer) {
    ssvs.clear();

    // Mobile Base SSVs
    SSV AB, CD1, CD2;

    // Segment AB
    Eigen::Vector3d base_pos = getLinkPosition(tf_buffer, "base_link");
    AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    AB.radius = 0.3;  // Adjusted radius as requested
    ssvs.push_back(AB);

    // Segment CD1
    CD1.point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
    CD1.point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
    CD1.radius = 0.09;
    ssvs.push_back(CD1);

    // Segment CD2
    CD2.point1 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 0.481);
    CD2.point2 = base_pos + Eigen::Vector3d(-0.2256, -0.07, 1.3);
    CD2.radius = 0.09;
    ssvs.push_back(CD2);

    // Manipulator SSVs
    SSV EF, FG, GH, I;

    // Use TF to get the transforms from the manipulator links
    Eigen::Vector3d panda_link0 = getLinkPosition(tf_buffer, "panda_link0");
    Eigen::Vector3d panda_link2 = getLinkPosition(tf_buffer, "panda_link2");

    EF.point1 = panda_link0;
    EF.point2 = panda_link2;
    EF.radius = 0.07;
    ssvs.push_back(EF);

    FG.point1 = panda_link2;
    FG.point2 = getLinkPosition(tf_buffer, "panda_link3");
    FG.radius = 0.07;
    ssvs.push_back(FG);

    GH.point1 = getLinkPosition(tf_buffer, "panda_link_4_1");
    GH.point2 = Eigen::Vector3d(0, 0.025, 0) + getLinkPosition(tf_buffer, "panda_link5");
    GH.radius = 0.09;
    ssvs.push_back(GH);

    I.point1 = getLinkPosition(tf_buffer, "panda_link7");
    I.point2 = getLinkPosition(tf_buffer, "panda_link8");
    I.radius = 0.1;
    ssvs.push_back(I);
}

// Create FCL collision objects
void createFCLObjects(std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects, const std::vector<SSV>& ssvs) {
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

// Load and create FCL collision object from an .obj file
std::shared_ptr<fcl::CollisionObjectd> createCollisionObjectFromMesh(const std::string& obj_file_path, const tf2_ros::Buffer& tf_buffer) {
    // Create an Assimp importer
    Assimp::Importer importer;

    // Load the mesh file
    const aiScene* scene = importer.ReadFile(obj_file_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes);
    if (!scene) {
        ROS_ERROR("Failed to load mesh file: %s", obj_file_path.c_str());
        return nullptr;
    }

    // Prepare FCL geometry from the mesh
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;

    for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
        aiMesh* mesh = scene->mMeshes[i];

        // Extract vertices
        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            aiVector3D vertex = mesh->mVertices[v];
            vertices.push_back(fcl::Vector3d(vertex.x, vertex.y, vertex.z));
        }

        // Extract triangles (faces)
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            aiFace face = mesh->mFaces[f];
            if (face.mNumIndices == 3) {  // Ensure it's a triangle
                triangles.push_back(fcl::Triangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]));
            }
        }
    }

    // Create FCL BVH model
    auto mesh_geometry = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    mesh_geometry->beginModel();
    mesh_geometry->addSubModel(vertices, triangles);
    mesh_geometry->endModel();

    // Get the transform of the base_link
    Eigen::Vector3d base_link_pos = getLinkPosition(tf_buffer, "base_link");
    fcl::Transform3d transform;
    transform.setIdentity();
    transform.translation() = base_link_pos;

    // Create the FCL collision object
    auto mesh_object = std::make_shared<fcl::CollisionObjectd>(mesh_geometry, transform);
    return mesh_object;
}

// Calculate the distance between two FCL objects
double calculateDistance(const std::shared_ptr<fcl::CollisionObjectd>& obj1, const std::shared_ptr<fcl::CollisionObjectd>& obj2, fcl::DistanceRequestd request, fcl::DistanceResultd result) {

    // Perform distance calculation
    double distance = fcl::distance(obj1.get(), obj2.get(), request, result);

    return distance;
}

// Find closest distance between SSVs
void findClosestSSVs(ros::Publisher& min_distance_pub, const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects, const std::vector<std::pair<int, int>>& ignorePairs) {
    double min_distance = std::numeric_limits<double>::max();
    double distance = 0;
    std::pair<int, int> closest_pair;
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;

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
            distance = calculateDistance(fcl_objects[i], fcl_objects[j], request, result);
            if (distance < min_distance) {
                min_distance = distance;
                closest_pair = {i, j};
            }
        }
    }

    if (min_distance <= 0) {
        // ROS_WARN_STREAM("Self-collision detected!");
    }

    // Publish the minimum distance
    std_msgs::Float32 distance_msg;
    distance_msg.data = min_distance;
    // min_distance_pub.publish(distance_msg);
}

// Find closest point on EE between base_link
void EEToBase(ros::Publisher& ee_pub, const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects) {
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    
    double distance = fcl::distance(fcl_objects[0].get(), fcl_objects[6].get(), request, result);

    if (result.nearest_points[0].isZero() || result.nearest_points[1].isZero()) {
        ROS_ERROR("Nearest points calculation failed. Got zero vectors.");
    } else {
        // ROS_WARN("Nearest point on EE: [%f, %f, %f]", result.nearest_points[1][0], result.nearest_points[1][1], result.nearest_points[1][2]);
        
        // Publish the nearest point (x, y, z) as geometry_msgs::Point
        geometry_msgs::Point ee_point;
        ee_point.x = result.nearest_points[1][0];
        ee_point.y = result.nearest_points[1][1];
        ee_point.z = result.nearest_points[1][2];
        
        ee_pub.publish(ee_point);  // Publish the full point
    }
}

// Publish distance between DBB and EE
void publishDistanceToDBB(ros::Publisher& dbb_distance_pub, ros::Publisher& ee_pub, const std::shared_ptr<fcl::CollisionObjectd>& obj_mesh, const std::vector<std::shared_ptr<fcl::CollisionObjectd>>& fcl_objects) {
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;  // Enable nearest points
    double distance = calculateDistance(obj_mesh, fcl_objects[6], request, result);

    if (distance <= 0) {
        EEToBase(ee_pub, fcl_objects);
    }

    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    // dbb_distance_pub.publish(distance_msg);
}


// Visualize FCL capsule
void publishFCLObjectsAsCollision(moveit_visual_tools::MoveItVisualTools& visual_tools, const std::vector<SSV>& ssvs) {
    visual_tools.deleteAllMarkers();

    for (const auto& ssv : ssvs) {
        visual_tools.publishCylinder(ssv.point1, ssv.point2, rviz_visual_tools::CYAN, ssv.radius * 2);
        visual_tools.publishSphere(ssv.point1, rviz_visual_tools::CYAN, ssv.radius * 2);
        visual_tools.publishSphere(ssv.point2, rviz_visual_tools::CYAN, ssv.radius * 2);
    }

    visual_tools.trigger();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ssv_collision_checker_fcl");
    ros::NodeHandle nh;

    // Initialize TF buffer and listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Load the .obj file and create FCL object for DBB
    std::string package_path = ros::package::getPath("mppi_royalpanda");
    std::string mesh_file_path = package_path + "/mesh/DBB_mesh.obj";
    std::shared_ptr<fcl::CollisionObjectd> obj_mesh = createCollisionObjectFromMesh(mesh_file_path, tf_buffer);

    if (!obj_mesh) {
        ROS_ERROR("Failed to create collision object from .obj file");
        return -1;
    }

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "/rviz_visual_tools");
    visual_tools.loadRemoteControl();

    // Publisher for minimum distance and dbb distance
    ros::Publisher min_distance_pub = nh.advertise<std_msgs::Float32>("/min_ssv_distance", 10);
    ros::Publisher dbb_distance_pub = nh.advertise<std_msgs::Float32>("/dbb_distance", 10);
    ros::Publisher ee_pub = nh.advertise<geometry_msgs::Point>("/ee_point", 10);

    std::vector<SSV> ssvs;
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> fcl_objects;
    ros::Rate rate(30);

    // Ignore SSV pairs already in collision
    std::vector<std::pair<int, int>> ignorePairs = {
        {0, 1},  // AB and CD1
        {0, 2},  // AB and CD2
        {0, 3},  // AB and EF
        {1, 2},  // CD1 and CD2
        {3, 4},  // EF and FG
        {4, 5},  // FG and GH
        {5, 6}   // GH and I
    };

    while (ros::ok()) {
        createSSVs(ssvs, tf_buffer);

        createFCLObjects(fcl_objects, ssvs);

        findClosestSSVs(min_distance_pub, fcl_objects, ignorePairs);

        if (obj_mesh) {
            publishDistanceToDBB(dbb_distance_pub, ee_pub, obj_mesh, fcl_objects);
        }

        // Visualize FCL objects in Rviz
        publishFCLObjectsAsCollision(visual_tools, ssvs);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
