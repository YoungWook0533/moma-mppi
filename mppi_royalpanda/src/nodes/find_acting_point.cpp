#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <fstream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

std::vector<std::tuple<Eigen::Vector3f, double>> dbb_points;  // Store DBB points globally

// Function to load and sort DBB points by z and score
std::vector<std::tuple<Eigen::Vector3f, double>> loadAndSortDBBPoints(const std::string& filename) {
    std::vector<std::tuple<Eigen::Vector3f, double>> points;
    std::ifstream infile(filename);
    float x, y, z, score;

    // Load DBB points from file
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open DBB file: %s", filename.c_str());
        return points;
    }

    int point_count = 0;
    while (infile >> x >> y >> z >> score) {
        points.emplace_back(Eigen::Vector3f(x, y, z), score);
        ++point_count;
    }
    infile.close();
    ROS_INFO("Successfully loaded %d points from DBB file.", point_count);

    // Sort points by z-coordinate first
    std::sort(points.begin(), points.end(), [](const auto& p1, const auto& p2) {
        return std::get<0>(p1).z() < std::get<0>(p2).z();
    });

    // Sort points with the same z-coordinate by score in descending order
    auto it = points.begin();
    while (it != points.end()) {
        auto range_end = std::upper_bound(it, points.end(), *it, [](const auto& p1, const auto& p2) {
            return std::get<0>(p1).z() < std::get<0>(p2).z();
        });
        std::sort(it, range_end, [](const auto& p1, const auto& p2) {
            return std::get<1>(p1) > std::get<1>(p2); // Sort by score descending
        });
        it = range_end;
    }

    return points;
}

// Function to publish DBB points as PointCloud2 with intensity
void publishDBBPointCloud(ros::Publisher& pointcloud_pub) {
    sensor_msgs::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.frame_id = "base_link";
    pointcloud_msg.header.stamp = ros::Time::now();
    pointcloud_msg.height = 1;
    pointcloud_msg.width = dbb_points.size();
    pointcloud_msg.is_dense = false;
    pointcloud_msg.is_bigendian = false;

    // Define the fields for the point cloud (x, y, z, intensity)
    sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(dbb_points.size());

    // Fill in the points with x, y, z coordinates and intensity
    sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(pointcloud_msg, "intensity");

    for (const auto& point : dbb_points) {
        *iter_x = std::get<0>(point).x();
        *iter_y = std::get<0>(point).y();
        *iter_z = std::get<0>(point).z();
        *iter_intensity = std::get<1>(point);  // Use score as intensity
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
    }

    // Publish the point cloud
    pointcloud_pub.publish(pointcloud_msg);
}


// Find points on DBB with the same z as the nearest point on EE
std::vector<std::tuple<Eigen::Vector3f, double>> findMatchingZPoints(
    const std::vector<std::tuple<Eigen::Vector3f, double>>& dbb_points, float ee_z, float threshold = 0.01) {

    std::vector<std::tuple<Eigen::Vector3f, double>> matching_points;

    // Iterate through all points and find those within the threshold for the z-coordinate
    for (const auto& point : dbb_points) {
        float z_coord = std::get<0>(point).z();
        if (fabs(z_coord - ee_z) <= threshold) {  // Check if within the threshold
            matching_points.push_back(point);
        }
    }

    return matching_points;
}

Eigen::Vector3f findActingPoint(
    const std::vector<std::tuple<Eigen::Vector3f, double>>& dbb_points, const Eigen::Vector3f& ee_position) {
    
    Eigen::Vector3f closest_point;
    double min_distance = std::numeric_limits<double>::max();

    // Step 1: Find the closest point on the DBB to the end-effector position
    for (const auto& point : dbb_points) {
        const Eigen::Vector3f& p = std::get<0>(point);
        double distance_to_ee = (p - ee_position).norm();

        if (distance_to_ee < min_distance) {
            min_distance = distance_to_ee;
            closest_point = p;
        }
    }

    // Step 2: Calculate step size α_i based on di and the score |S(p)|
    double max_score = -std::numeric_limits<double>::max();
    Eigen::Vector3f acting_point = closest_point;

    for (const auto& point : dbb_points) {
        const Eigen::Vector3f& p = std::get<0>(point);
        double score = std::get<1>(point);

        double distance_to_closest = (p - closest_point).norm();
        double alpha_i = min_distance / std::abs(score / 300);  // Step size

        // Step 3: Check if the point falls within the step radius and has the highest score
        if (distance_to_closest <= alpha_i && score > max_score) {
            max_score = score;
            acting_point = p;
        }
    }

    return acting_point;
}


// Publish a marker at the acting point
void publishMarker(ros::Publisher& marker_pub, const Eigen::Vector3f& acting_point) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "acting_point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = acting_point.x();
    marker.pose.position.y = acting_point.y();
    marker.pose.position.z = acting_point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;  // Adjust the size of the sphere
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;  // Red marker
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_pub.publish(marker);
}

// Callback function for the /ee_points topic
void eePointsCallback(const geometry_msgs::Point::ConstPtr& msg, ros::Publisher& marker_pub) {
    ROS_INFO("Received /ee_points message: [%f, %f, %f]", msg->x, msg->y, msg->z);

    Eigen::Vector3f ee_position(msg->x, msg->y, msg->z);
    float ee_z = msg->z;

    // Find points on DBB with the same z as the received z-coordinate
    std::vector<std::tuple<Eigen::Vector3f, double>> matching_points = findMatchingZPoints(dbb_points, ee_z);

    if (matching_points.empty()) {
        ROS_ERROR("No matching points found on DBB with z-coordinate %f", ee_z);
        return;
    } else {
        ROS_INFO("Found %lu matching points with z-coordinate %f", matching_points.size(), ee_z);
    }

    // Find the acting point with the highest score considering distance from EE
    Eigen::Vector3f acting_point = findActingPoint(matching_points, ee_position);
    ROS_INFO("Acting Point on DBB: [%f, %f, %f]", acting_point.x(), acting_point.y(), acting_point.z());

    // Publish the marker at the acting point
    publishMarker(marker_pub, acting_point);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dbb_acting_point_finder");
    ros::NodeHandle nh;

    // Load and sort DBB points once when the node starts
    std::string package_path = ros::package::getPath("mppi_royalpanda");
    std::string txt_file_path = package_path + "/mesh/DBB.txt";
    dbb_points = loadAndSortDBBPoints(txt_file_path);

    if (dbb_points.empty()) {
        ROS_ERROR("DBB points failed to load. Exiting node.");
        return -1;
    }

    // Publisher for the marker
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("acting_point_marker", 10);
    
    // Publisher for DBB points as point cloud
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("dbb_pointcloud", 10);
    // Subscribe to /ee_points to receive the x, y, z coordinates of the nearest point on the EE
    ros::Subscriber ee_points_sub = nh.subscribe<geometry_msgs::Point>("/ee_point", 10, boost::bind(eePointsCallback, _1, boost::ref(marker_pub)));
    
    ros::Rate rate(10);

    while (ros::ok()) {

        publishDBBPointCloud(pointcloud_pub);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
