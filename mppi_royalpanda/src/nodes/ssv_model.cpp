#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <limits>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

// Define a structure for SSV (Sphere Swept Volume)
struct SSV {
    Eigen::Vector3d point1;  // Start point of the line segment
    Eigen::Vector3d point2;  // End point of the line segment
    double radius;           // Radius of the sphere
};

// Utility function to print SSV information
void printSSV(const SSV& ssv, const std::string& name) {
    ROS_INFO_STREAM(name << " from (" << ssv.point1.transpose() << ") to (" << ssv.point2.transpose() << "), radius = " << ssv.radius);
}

// Fetch a transform from TF2 and convert it to Eigen::Vector3d
Eigen::Vector3d getLinkPosition(const tf2_ros::Buffer& tf_buffer, const std::string& link_name) {
    try {
        geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("base_link", link_name, ros::Time(0));
        return tf2::transformToEigen(transformStamped.transform).translation();
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return Eigen::Vector3d::Zero();  // Return default vector if not found
    }
}

// Visualization function for RViz
void publishSSVMarker(ros::Publisher& marker_pub, int id, const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius, const std::string& frame_id) {
    // Define marker for the cylinder (line segment)
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ssv";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    // Calculate the midpoint of the line segment
    Eigen::Vector3d mid_point = (start + end) / 2.0;
    marker.pose.position.x = mid_point.x();
    marker.pose.position.y = mid_point.y();
    marker.pose.position.z = mid_point.z();

    // Calculate the orientation of the cylinder
    Eigen::Vector3d dir = (end - start).normalized();
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);  // Default axis of the cylinder

    // Compute rotation axis and angle between z_axis and direction vector
    Eigen::Vector3d rotation_axis = z_axis.cross(dir);
    double rotation_angle = acos(z_axis.dot(dir));

    tf2::Quaternion q;
    if (rotation_axis.norm() > 1e-8) {  // Avoid singularities
        rotation_axis.normalize();
        q.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), rotation_angle);
    } else {
        // If the vectors are collinear, no rotation is needed (keep default orientation)
        q.setRPY(0, 0, 0);
    }

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Set the dimensions of the cylinder
    marker.scale.x = radius * 2.0;  // diameter
    marker.scale.y = radius * 2.0;
    marker.scale.z = (start - end).norm();  // length of the line segment

    // Color the cylinder
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // Publish the marker
    marker_pub.publish(marker);

    // Define and publish the start and end spheres
    visualization_msgs::Marker start_sphere = marker;
    start_sphere.id = id + 1000;  // unique ID
    start_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.pose.position.x = start.x();
    start_sphere.pose.position.y = start.y();
    start_sphere.pose.position.z = start.z();
    start_sphere.scale.x = radius * 2.0;
    start_sphere.scale.y = radius * 2.0;
    start_sphere.scale.z = radius * 2.0;
    marker_pub.publish(start_sphere);

    visualization_msgs::Marker end_sphere = start_sphere;
    end_sphere.id = id + 2000;  // unique ID
    end_sphere.pose.position.x = end.x();
    end_sphere.pose.position.y = end.y();
    end_sphere.pose.position.z = end.z();
    marker_pub.publish(end_sphere);
}

// Ensure proper TF usage and avoid issues with ignored pairs
void createSSVs(std::vector<SSV>& ssvs, const tf2_ros::Buffer& tf_buffer) {
    // Base SSVs
    SSV AB, CD1, CD2;

    // Segment AB (base link points A and B)
    Eigen::Vector3d base_pos = getLinkPosition(tf_buffer, "base_link");
    AB.point1 = base_pos + Eigen::Vector3d(0.2776, 0, 0.2405);
    AB.point2 = base_pos + Eigen::Vector3d(-0.2776, 0, 0.2405);
    AB.radius = 0.3;  // Adjusted radius as requested
    ssvs.push_back(AB);

    // Segment CD1 (base link points C1 and D1)
    CD1.point1 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 0.481);
    CD1.point2 = base_pos + Eigen::Vector3d(-0.2256, 0.07, 1.3);
    CD1.radius = 0.09;
    ssvs.push_back(CD1);

    // Segment CD2 (base link points C2 and D2)
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

// Calculate the distance between two line segments using parametric equation
double calculateDistance(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
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

// Correct distance calculation and ignoring already known collisions
void findClosestSSVs(ros::Publisher& min_distance_pub, const std::vector<SSV>& ssvs) {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, int> closest_pair;

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
                closest_pair = {i, j};
            }
        }
    }

    ROS_INFO_STREAM("Closest SSV pair: " << closest_pair.first << " and " << closest_pair.second << " with distance = " << min_distance);
    
    // Warn if self-collision occures
    if(min_distance <= 0){
        ROS_WARN_STREAM("Self collision occured!");
    }

    // Publish the minimum distance
    std_msgs::Float32 distance_msg;
    distance_msg.data = min_distance;
    min_distance_pub.publish(distance_msg);
}

void deleteSSVMarkers(ros::Publisher& marker_pub, int id_start, int num_markers, const std::string& frame_id) {
    for (int i = id_start; i < id_start + num_markers; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "ssv";
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE; 
        marker_pub.publish(marker);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ssv_collision_checker");
    ros::NodeHandle nh;

    // TF listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // ROS publisher for RViz markers
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("ssv_markers", 1);

    // ROS publisher for the minimum distance
    ros::Publisher min_distance_pub = nh.advertise<std_msgs::Float32>("min_link_distance", 10);

    // Create a vector to store the SSVs
    std::vector<SSV> ssvs;

    // Number of markers used to track the IDs for deletion
    int num_markers = 0;

    // Continuously calculate the minimum distances
    ros::Rate rate(30);
    while (ros::ok()) {
        ssvs.clear();

        // Delete previously published markers
        deleteSSVMarkers(marker_pub, 0, num_markers, "base_link");

        createSSVs(ssvs, tf_buffer);

        // Print the SSVs and publish the markers
        num_markers = ssvs.size();  // Update the number of markers
        for (size_t i = 0; i < ssvs.size(); ++i) {
            publishSSVMarker(marker_pub, i, ssvs[i].point1, ssvs[i].point2, ssvs[i].radius, "base_link");
        }

        // Find and log the closest SSVs
        findClosestSSVs(min_distance_pub, ssvs);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}