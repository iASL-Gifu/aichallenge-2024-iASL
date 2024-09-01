#include "path_publisher.hpp"
#include <fstream>
#include <sstream>
#include <string>

namespace path_publisher
{

PathPublisher::PathPublisher() 
    : Node("path_publihser"), path_initialized_(false), current_section_(-1), prev_x_(0.0), prev_y_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "================ Path Publisher ==================");

    this->declare_parameter("raceline_csv_path", std::string("raceline_csv_path"));
    raceline_csv_path_ = this->get_parameter("raceline_csv_path").as_string();

    this->declare_parameter("centerline_csv_path", std::string("centerline_csv_path"));
    centerline_csv_path_ = this->get_parameter("centerline_csv_path").as_string();

    this->declare_parameter("raceline_downsample_rate", 10);
    raceline_downsample_rate_ = this->get_parameter("raceline_downsample_rate").as_int();

    this->declare_parameter("centerline_downsample_rate", 10);
    centerline_downsample_rate_ = this->get_parameter("centerline_downsample_rate").as_int();

    this->declare_parameter("loop_count", 10);
    loop_count_ = this->get_parameter("loop_count").as_int();

    this->declare_parameter("publish_rate", 10);
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    this->declare_parameter("margin", 10);
    margin_ = this->get_parameter("margin").as_int();

    RCLCPP_INFO(this->get_logger(), "csv_path: %s", raceline_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "csv_path: %s", centerline_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "downsample_rate: %d", centerline_downsample_rate_);
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

    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aichallenge/objects", 1, 
        std::bind(&PathPublisher::object_callback, this, std::placeholders::_1)
    );

    get_path_srv_ = this->create_service<path_service::srv::GetPath>(
        "get_path", 
        std::bind(&PathPublisher::handle_get_path, this, std::placeholders::_1, std::placeholders::_2)
    );

    // for (int section = 0; section < 8; section++) {
    //     std::string service_name = "/obstacle_path/" + std::to_string(section);
    //     auto client = this->create_client<path_service::srv::GetObstaclePath>(service_name);
    //     get_obstacle_path_clients_.push_back(client);
    // }

    load_csv(raceline_csv_path_, raceline_downsample_rate_, raceline_points_);
    load_csv(centerline_csv_path_, centerline_downsample_rate_, centerline_points_);
}

void PathPublisher::load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point) {
    RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

    std::ifstream file(csv_path);
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
            
            if (line_count_ % downsample_rate != 0)
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

            point.push_back(pose);
        }
        file.close();
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points", point.size());
}

void PathPublisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "x_x_x_x_x_x_x_x_x_x_x_x_x_x_x");
    odometry_ = *msg;

    if (path_initialized_) return;

    double start_x = odometry_.pose.pose.position.x;
    double start_y = odometry_.pose.pose.position.y;

    double min_start_dist = std::numeric_limits<double>::infinity();
    int min_start_index = 0;
    for (int i = 0; i < raceline_points_.size(); i++) {
        double dist = std::sqrt(
            std::pow(raceline_points_[i].pose.position.x - start_x, 2) +
            std::pow(raceline_points_[i].pose.position.y - start_y, 2)
        );

        if (dist < min_start_dist) {
            min_start_index = i;
            min_start_dist = dist;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Closest waypoint to start is %d", min_start_index);
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", raceline_points_[min_start_index].pose.position.x, raceline_points_[min_start_index].pose.position.x);

    // min_start_indexが先頭になるpathを生成
    std::vector<geometry_msgs::msg::PoseStamped> new_points;
    for (int i = min_start_index; i < raceline_points_.size(); i++) {
        new_points.push_back(raceline_points_[i]);
    }
    for (int i = 0; i < min_start_index; i++) {
        new_points.push_back(raceline_points_[i]);
    }

    std::vector<geometry_msgs::msg::PoseStamped> points;

    for (int i = 0; i < loop_count_; i++) {
        for (int j = 0; j < new_points.size(); j++) {
            points.push_back(new_points[j]);
        }
    }

    path_initialized_ = true;
    current_index_ = 0;

    nav_msgs::msg::Path path;
    path.poses.clear();
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    path.poses = points;

    path_pub_->publish(path);
}

void PathPublisher::object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<double> data = msg->data;

    int last_index = data.size() - 4;
    if (last_index < 0) return;

    double object_x = data[last_index + 0];
    double object_y = data[last_index + 1];

    RCLCPP_INFO(this->get_logger(), "object_x: %f, object_y: %f", object_x, object_y);

    if (prev_x_ == object_x && prev_y_ == object_y) return;

    // auto request = std::make_shared<path_service::srv::GetObstaclePath::Request>();
    // request.x = object_x;
    // request.y = object_y;

    // using ServiceResponseFuture = rclcpp::Client<path_service::srv::GetObstaclePath>::SharedFuture;

    // auto response_callback = [this](ServiceResponseFuture future) {
    //     auto result = future.get();
    //     nav_msgs::msg::Path obstacle_path = result->path;
    // }

    // get_obstacle_path_clients_[current_section_]->async_send_request(request, response_callback);

    current_section_ = (current_section_ + 1) % 8;
    prev_x_ = object_x;
    prev_y_ = object_y;

    RCLCPP_INFO(this->get_logger(), "yyyyyyyyyyyyyy %d", current_section_);
}

void PathPublisher::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    double x = msg->poses[0].pose.position.x;
    double y = msg->poses[0].pose.position.y;

    // RCLCPP_INFO(this->get_logger(), "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
    // RCLCPP_INFO(this->get_logger(), "Index 0 Position: x: %f, y: %f", x, y);

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

    // RCLCPP_INFO(this->get_logger(), "Index 0 Position: x: %f, y: %f", new_path.poses[0].pose.position.x, new_path.poses[0].pose.position.y);

    path_pub_->publish(new_path);
}

void PathPublisher::handle_get_path(const std::shared_ptr<path_service::srv::GetPath::Request> request,
                                    std::shared_ptr<path_service::srv::GetPath::Response> response)
{
    std::string csv_path = request->csv_path;
    nav_msgs::msg::Path path;

    if (csv_path == "centerline") {
        path.poses = centerline_points_;
    } else if (csv_path == "raceline") {
        path.poses = raceline_points_;
    }

    path.header.stamp = this->now();
    path.header.frame_id = "map";

    response->path = path;
    RCLCPP_INFO(this->get_logger(), "Path sent to client with %ld points.", path.poses.size());
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_publisher::PathPublisher>());
    rclcpp::shutdown();
    return 0;
}