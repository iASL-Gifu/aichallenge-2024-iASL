#include "path_publisher.hpp"
#include <fstream>
#include <sstream>
#include <string>

namespace path_publisher
{

PathPublisher::PathPublisher() 
    : Node("path_publihser"), csv_loaded_(false), path_initialized_(false), odometry_received_(false)
{
    RCLCPP_INFO(this->get_logger(), "================ Path Publisher ==================");

    this->declare_parameter("csv_path", std::string("csv_path"));
    csv_path_ = this->get_parameter("csv_path").as_string();

    this->declare_parameter("downsample_rate", 10);
    downsample_rate_ = this->get_parameter("downsample_rate").as_int();

    this->declare_parameter("loop_count", 10);
    loop_count_ = this->get_parameter("loop_count").as_int();

    this->declare_parameter("publish_rate", 10);
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    this->declare_parameter("margin", 10);
    margin_ = this->get_parameter("margin").as_int();

    if (csv_path_.empty()) {
        RCLCPP_INFO(this->get_logger(), "csv_path is not set");
    }
    RCLCPP_INFO(this->get_logger(), "csv_path: %s", csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "downsaple_rate: %d", downsample_rate_);
    RCLCPP_INFO(this->get_logger(), "loop_count: %d", loop_count_);
    RCLCPP_INFO(this->get_logger(), "publish_rate: %d", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "margin: %d", margin_);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "localization/kinematic_state", 1,
        std::bind(&PathPublisher::odometry_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vechile_path", 1);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "obstacle_avoidance", 1, std::bind(&PathPublisher::path_callback, this, std::placeholders::_1)
    );

    
    RCLCPP_INFO(this->get_logger(), "--------------- Load CSV ---------------");

    std::ifstream file(csv_path_);
    std::string line;
    line_count_ = 0;
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
    } else {
        RCLCPP_INFO(this->get_logger(), "Reading CSV file");
        std::getline(file, line);
        while (std::getline(file, line))
        {
            line_count_++;
            
            if (line_count_ % downsample_rate_ != 0)
            {
                continue;
            }

            std::stringstream ss(line);
            std::string x, y, z, x_quat, y_quat, z_quat, w_quat;
            std::getline(ss, x, ',');
            std::getline(ss, y, ',');
            std::getline(ss, z, ',');
            std::getline(ss, x_quat, ',');
            std::getline(ss, y_quat, ',');
            std::getline(ss, z_quat, ',');
            std::getline(ss, w_quat, ',');

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = std::stof(x);
            pose.pose.position.y = std::stof(y);
            pose.pose.position.z = 43.1;
            pose.pose.orientation.x = std::stof(x_quat);
            pose.pose.orientation.y = std::stof(y_quat);
            pose.pose.orientation.z = std::stof(z_quat);
            pose.pose.orientation.w = std::stof(w_quat);

            pose.header.frame_id = "map";
            pose.header.stamp = this->now();

            points_.push_back(pose);
        }
        file.close();
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %d points", line_count_);

    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(publish_rate_),
    //     std::bind(&PathPublisher::on_timer, this));
}

void PathPublisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "x_x_x_x_x_x_x_x_x_x_x_x_x_x_x");
    odometry_ = *msg;
    odometry_received_ = true;

    if (path_initialized_) return;

    double start_x = odometry_.pose.pose.position.x;
    double start_y = odometry_.pose.pose.position.y;

    double min_start_dist = std::numeric_limits<double>::infinity();
    int min_start_index = 0;
    for (int i = 0; i < points_.size(); i++) {
        double dist = std::sqrt(
            std::pow(points_[i].pose.position.x - start_x, 2) +
            std::pow(points_[i].pose.position.y - start_y, 2)
        );

        if (dist < min_start_dist) {
            min_start_index = i;
            min_start_dist = dist;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Closest waypoint to start is %d", min_start_index);

    // min_start_indexが先頭になるpathを生成
    std::vector<geometry_msgs::msg::PoseStamped> new_points;
    for (int i = min_start_index; i < points_.size(); i++) {
        new_points.push_back(points_[i]);
    }
    for (int i = 0; i < min_start_index; i++) {
        new_points.push_back(points_[i]);
    }

    // loop the points
    points_.clear();

    for (int i = 0; i < loop_count_; i++) {
        for (int j = 0; j < new_points.size(); j++) {
            points_.push_back(new_points[j]);
        }
    }

    path_initialized_ = true;
    current_index_ = 0;

    nav_msgs::msg::Path path;
    path.poses.clear();
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    path.poses = points_;

    path_pub_->publish(path);
}

void PathPublisher::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    double x = msg->poses[0].pose.position.x;
    double y = msg->poses[0].pose.position.y;

    RCLCPP_INFO(this->get_logger(), "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
    RCLCPP_INFO(this->get_logger(), "Index 0 Position: x: %f, y: %f", x, y);

    nav_msgs::msg::Path new_path;
    new_path.poses.clear();
    new_path.header.stamp = this->now();
    new_path.header.frame_id = "map";

    double start_x = odometry_.pose.pose.position.x;
    double start_y = odometry_.pose.pose.position.y;
    double min_dist = std::numeric_limits<double>::infinity();
    int index = 0;
    for (int i = 0; i < margin_; i++) {
        double dist = std::sqrt(
            std::pow(msg->poses[i].pose.position.x - start_x, 2) + 
            std::pow(msg->poses[i].pose.position.y - start_y, 2)
        );

        if (dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }

    new_path.poses.assign(msg->poses.begin() + index, msg->poses.end());

    RCLCPP_INFO(this->get_logger(), "Index 0 Position: x: %f, y: %f", new_path.poses[0].pose.position.x, new_path.poses[0].pose.position.y);

    path_pub_->publish(new_path);
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_publisher::PathPublisher>());
    rclcpp::shutdown();
    return 0;
}