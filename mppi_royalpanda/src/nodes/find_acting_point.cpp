#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <ros/package.h>
#include <math.h>

const double scaling = 0.3;  // Linear velocity scaling
const double command_timeout = 0.05;   // Timeout for stopping (in seconds)

std::vector<std::tuple<Eigen::Vector3f, double>> dbb_points;
Eigen::Vector3f last_acting_point;  // Store the previous acting point for smooth transition
bool first_search = true;  // Flag to check if it's the first acting point selection
float max_transition_distance = 0.1;  // Define a bound for the maximum allowed transition

// ros::Publisher velocity_pub;
ros::Publisher marker_pub;
ros::Publisher pointcloud_pub;
ros::Publisher acting_point_pub;
ros::Timer stop_timer;

std::vector<std::tuple<Eigen::Vector3f, double>> loadAndSortDBBPoints(const std::string& filename) {
    std::vector<std::tuple<Eigen::Vector3f, double>> points;
    std::ifstream infile(filename);
    float x, y, z, score;

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

    // First, sort by z, then x, then y, in ascending order
    std::sort(points.begin(), points.end(), [](const auto& p1, const auto& p2) {
        const auto& p1_pos = std::get<0>(p1);
        const auto& p2_pos = std::get<0>(p2);
        if (p1_pos.z() != p2_pos.z()) return p1_pos.z() < p2_pos.z();
        if (p1_pos.x() != p2_pos.x()) return p1_pos.x() < p2_pos.x();
        return p1_pos.y() < p2_pos.y();
    });

    // Then, sort by score within each (z, x, y) group in descending order
    auto it = points.begin();
    while (it != points.end()) {
        auto range_end = std::upper_bound(it, points.end(), *it, [](const auto& p1, const auto& p2) {
            const auto& p1_pos = std::get<0>(p1);
            const auto& p2_pos = std::get<0>(p2);
            return p1_pos.z() < p2_pos.z() ||
                   (p1_pos.z() == p2_pos.z() && p1_pos.x() < p2_pos.x()) ||
                   (p1_pos.z() == p2_pos.z() && p1_pos.x() == p2_pos.x() && p1_pos.y() < p2_pos.y());
        });
        
        // Sort within the group by score
        std::sort(it, range_end, [](const auto& p1, const auto& p2) {
            return std::get<1>(p1) > std::get<1>(p2);
        });

        it = range_end;
    }

    return points;
}


// Function to publish DBB points as PointCloud2 with intensity
void publishDBBPointCloud() {
    sensor_msgs::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.frame_id = "robot_base_link";
    pointcloud_msg.header.stamp = ros::Time::now();
    pointcloud_msg.height = 1;
    pointcloud_msg.width = dbb_points.size();
    pointcloud_msg.is_dense = false;
    pointcloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(dbb_points.size());

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

    pointcloud_pub.publish(pointcloud_msg);
}

Eigen::Vector3f findActingPoint(const std::vector<std::tuple<Eigen::Vector3f, double>>& dbb_points, const Eigen::Vector3f& ee_position) {
    Eigen::Vector3f closest_point;
    double min_distance = std::numeric_limits<double>::max();

    // Step 1: Find the closest point to the EE position
    for (const auto& point : dbb_points) {
        const Eigen::Vector3f& p = std::get<0>(point);
        double distance_to_ee = (p - ee_position).norm();

        if (distance_to_ee < min_distance) {
            min_distance = distance_to_ee;
            closest_point = p;
        }
    }

    // Step 2: Define x and y bounds based on the score-dependent alpha values
    std::vector<std::tuple<Eigen::Vector3f, double>> candidates;
    for (const auto& point : dbb_points) {
        const Eigen::Vector3f& p = std::get<0>(point);
        double score = std::get<1>(point);

        double alpha_x = min_distance / std::abs(score / 100.0);
        double alpha_y = min_distance / std::abs(score / 100.0);

        // Only add points within the x and y bounds relative to closest_point to candidates
        if (std::abs(p.x() - closest_point.x()) <= alpha_x && std::abs(p.y() - closest_point.y()) <= alpha_y) {
            candidates.push_back(point);
        }
    }

    // Step 3: From the candidates, find the point with the highest score
    double max_score = -std::numeric_limits<double>::max();
    Eigen::Vector3f acting_point = closest_point;

    for (const auto& candidate : candidates) {
        const Eigen::Vector3f& p = std::get<0>(candidate);
        double score = std::get<1>(candidate);

        // Update acting_point if the candidate has a higher score
        if (score > max_score) {
            if (first_search || (p - last_acting_point).norm() <= max_transition_distance) {
                max_score = score;
                acting_point = p;
            }
        }
    }

    last_acting_point = acting_point;  // Update last acting point
    first_search = false;              // Update flag after first search
    return acting_point;
}




// Publish a marker at the acting point and the acting point coordinates
void publishActingPoint(const Eigen::Vector3f& acting_point) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "robot_base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "acting_point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = acting_point.x();
    marker.pose.position.y = acting_point.y();
    marker.pose.position.z = acting_point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_pub.publish(marker);

    // Publish the acting point as geometry_msgs::Point
    geometry_msgs::Point point_msg;
    point_msg.x = acting_point.x();
    point_msg.y = acting_point.y();
    point_msg.z = acting_point.z();
    acting_point_pub.publish(point_msg);
}

// Callback function for the /ee_points topic
void eePointsCallback(const geometry_msgs::Point::ConstPtr& msg) {
    ROS_INFO("Received /ee_points message: [%f, %f, %f]", msg->x, msg->y, msg->z);

    Eigen::Vector3f ee_position(msg->x, msg->y, msg->z);
    Eigen::Vector3f acting_point = findActingPoint(dbb_points, ee_position);
    ROS_INFO("Acting Point on DBB: [%f, %f, %f]", acting_point.x(), acting_point.y(), acting_point.z());

    publishActingPoint(acting_point);
}

// Callback function to process acting point data and publish velocity commands
void actingPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    geometry_msgs::Twist cmd_vel;

    double x = msg->x;
    double y = msg->y;

    double distance_to_origin = std::sqrt(x * x + y * y);
    double target_angle = std::atan2(y, x);

    // Set linear velocity
    cmd_vel.linear.x = -scaling * distance_to_origin;

    // Set angular velocity
    cmd_vel.angular.z = scaling * target_angle;

    ROS_WARN("Target vel : %f, %f", -scaling * distance_to_origin, scaling * target_angle);

    // Publish the velocity command
    // velocity_pub.publish(cmd_vel);

    // Reset the stop timer to avoid stopping if valid commands are received
    stop_timer.stop();
    stop_timer.start();
}

// Function to stop the robot by publishing zero velocities
void stopRobot(const ros::TimerEvent&) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    // velocity_pub.publish(stop_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "acting_point_finder_and_controller");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("mppi_royalpanda");
    std::string txt_file_path = package_path + "/mesh/DBB.txt";
    dbb_points = loadAndSortDBBPoints(txt_file_path);

    if (dbb_points.empty()) {
        ROS_ERROR("DBB points failed to load. Exiting node.");
        return -1;
    }

    // Publishers
    marker_pub = nh.advertise<visualization_msgs::Marker>("acting_point_marker", 10);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("dbb_pointcloud", 10);
    acting_point_pub = nh.advertise<geometry_msgs::Point>("acting_point", 10);
    // velocity_pub = nh.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel1", 10);

    // Subscriber to /ee_point topic
    ros::Subscriber ee_points_sub = nh.subscribe<geometry_msgs::Point>("/ee_point", 10, eePointsCallback);

    // Subscriber to /acting_point topic for controlling the robot
    ros::Subscriber acting_point_sub = nh.subscribe<geometry_msgs::Point>("/acting_point", 10, actingPointCallback);

    // Timer to stop the robot if no acting point is received within the timeout period
    stop_timer = nh.createTimer(ros::Duration(command_timeout), stopRobot, true);

    ros::Rate rate(100);

    while (ros::ok()) {
        publishDBBPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}