#include "path_publisher.hpp"
#include <fstream>
#include <sstream>
#include <string>

namespace path_publisher
{

PathPublisher::PathPublisher() 
    : Node("path_publihser"), current_section_(0), section_(0)
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

    this->declare_parameter("publish_rate", 10);
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    this->declare_parameter("margin", 10);
    margin_ = this->get_parameter("margin").as_int();

    this->declare_parameter("section_point", std::vector<double>());
    section_point_ = this->get_parameter("section_point").as_double_array();

    this->declare_parameter("start_index", 10);
    start_index_ = this->get_parameter("start_index").as_int();

    for (int i = 0; i < 8; i++) {
        double x = section_point_[i * 2 + 0];
        double y = section_point_[i * 2 + 1];
        RCLCPP_INFO(this->get_logger(), "Section x: %f, y: %f", x, y);
    }

    RCLCPP_INFO(this->get_logger(), "csv_path: %s", raceline_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "csv_path: %s", centerline_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "downsample_rate: %d", centerline_downsample_rate_);
    RCLCPP_INFO(this->get_logger(), "publish_rate: %d", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "margin: %d", margin_);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "localization/kinematic_state", 1,
        std::bind(&PathPublisher::odometry_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vechile_path", 1);

    // path2_pub_ = this->create_publisher<nav_msgs::msg::Path>("example", 1);

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

    for (int section = 0; section < 8; section++) {
        std::string service_name = "/obstacle_path_" + std::to_string(section);
        auto client = this->create_client<path_service::srv::GetObstaclePath>(service_name);
        get_obstacle_path_clients_.push_back(client);
    }

    load_csv(centerline_csv_path_, centerline_downsample_rate_, centerline_points_);
    load_csv(raceline_csv_path_, raceline_downsample_rate_, raceline_points_);

    objects_coordinate_ = std::vector<std::pair<double, double>>(8, std::make_pair(0.0, 0.0));
    section_path_ = std::vector<std::vector<geometry_msgs::msg::PoseStamped>>(8);
    path_state_ = std::vector<bool>(8, false);
    object_state_ = std::vector<bool>(8, false);
    add_path_ = std::vector<bool>(8, false);

    obstacle_path_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), // 実行間隔を秒単位で設定
        std::bind(&PathPublisher::send_obstacle_path_request, this)
    );
}

void PathPublisher::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "===== Subscribe Obstacle Path =====");
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

    if (add_path_[section_]) {
        RCLCPP_INFO(this->get_logger(), "VVVVVVVVVVVVVVVVVVVVVVVVVV");
        for (int i = 0; i < path_.poses.size() - 1; i++) {
            new_path.poses.push_back(path_.poses[i]);
        }
        add_path_[section_] = false;
        path_state_[section_] = false;
        object_state_[section_] = false;
        section_ = (section_ + 1) % 8;
    }

    path_pub_->publish(new_path);
}

void PathPublisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry_ = *msg;
}

void PathPublisher::object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<double> data = msg->data;

    int index = current_section_ * 4;
    if (index == data.size()) return;
    double object_x = data[index + 0];
    double object_y = data[index + 1];
    double ex_x = objects_coordinate_[current_section_].first;
    double ex_y = objects_coordinate_[current_section_].second;

    if (object_x == ex_x && object_y == ex_y) {
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Path Publisher Client");
    // RCLCPP_INFO(this->get_logger(), "Server Object x: %f, y: %f", object_x, object_y);

    objects_coordinate_[current_section_].first = object_x;
    objects_coordinate_[current_section_].second = object_y;

    object_state_[current_section_] = true;
    path_state_[current_section_] = true;
    current_section_ = (current_section_ + 1) % 8;
}

void PathPublisher::send_obstacle_path_request() {
    if (path_state_[section_] && !path_state_[(section_ - 1) % 8] && object_state_[section_]) {
        RCLCPP_INFO(this->get_logger(), "TTTTTTTTTTTTTTTTTTTTT %d", section_);
        auto request = std::make_shared<path_service::srv::GetObstaclePath::Request>();
        double object_x = objects_coordinate_[section_].first;
        double object_y = objects_coordinate_[section_].second;
        request->x = object_x;
        request->y = object_y;

        using ServiceResponseFuture = rclcpp::Client<path_service::srv::GetObstaclePath>::SharedFuture;
        
        auto response_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            path_ = result->path;
            add_path_[section_] = true;
            RCLCPP_INFO(this->get_logger(), "%dClient Path size: %zu", section_, path_.poses.size());
        };
        get_obstacle_path_clients_[section_]->async_send_request(request, response_callback);

        object_state_[section_] = false;
    }
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
    } else if (csv_path == "optimize") {
        path.poses = optimize_points_;
    }else {
        int section_index = std::stoi(csv_path); // CSV パスを整数に変換

        if (section_path_[section_index].size() > 0) {
            path.poses = section_path_[section_index];
        }
    }

    path.header.stamp = this->now();
    path.header.frame_id = "map";

    response->path = path;
    RCLCPP_INFO(this->get_logger(), "Path sent to client with %ld points.", path.poses.size());
}

void PathPublisher::load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point) {
    RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

    std::ifstream file(csv_path);
    std::string line;
    int line_count = 0;
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
    } else {
        RCLCPP_INFO(this->get_logger(), "Reading CSV file");
        std::getline(file, line);
        while (std::getline(file, line))
        {
            line_count++;
            
            if (line_count % downsample_rate != 0)
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

    if (csv_path == raceline_csv_path_) {
        // nav_msgs::msg::Path new_path;
        // new_path.poses.clear();
        // new_path.header.stamp = this->now();
        // new_path.header.frame_id = "map";
        // new_path.poses = raceline_points_;

        // RCLCPP_INFO(this->get_logger(), "xxxxxxxxxxxxxxxxxxxxxxxxxx");
        // path_pub_->publish(new_path);

        int x = 5;
        timer_ = this->create_wall_timer(
            std::chrono::seconds(x),
            std::bind(&PathPublisher::divide_section, this)
        );
    }
}

void PathPublisher::divide_section() {
    RCLCPP_INFO(this->get_logger(), "================ Path Divide ==================");

    std::vector<geometry_msgs::msg::PoseStamped> points;
    for (int i = start_index_; i < raceline_points_.size(); i++) {
        points.push_back(raceline_points_[i]);
    }
    for (int i = 0; i < start_index_; i++) {
        points.push_back(raceline_points_[i]);
    }

    optimize_points_ = points;

    double start_x = section_point_[2 * 7 + 0];
    double start_y = section_point_[2 * 7 + 1];

    int start_index = 0;
    int end_index = 0;

    for (int i = 0; i < 8; i++) {
        double end_x = section_point_[2 * i + 0];
        double end_y = section_point_[2 * i + 1];

        double min_start_dist = std::numeric_limits<double>::infinity();
        double min_end_dist = std::numeric_limits<double>::infinity();

        for (int j = 0; j < points.size(); j++) {
            double start_dist = std::sqrt(
                std::pow(start_x - points[j].pose.position.x, 2) +
                std::pow(start_y - points[j].pose.position.y, 2)
            );

            if (start_dist < min_start_dist) {
                start_index = j;
                min_start_dist = start_dist;
            }

            double end_dist = std::sqrt(
                std::pow(end_x - points[j].pose.position.x, 2) +
                std::pow(end_y - points[j].pose.position.y, 2)
            );

            if (end_dist < min_end_dist) {
                if (j == 0) end_index = 343;
                else end_index = j;
                min_end_dist = end_dist;
            }
        }

        RCLCPP_INFO(this->get_logger(), "start_index: %d, end_index: %d", start_index, end_index);

        section_path_[i].assign(points.begin() + start_index, points.begin() + end_index + 1);

        start_x = end_x;
        start_y = end_y;      
    }

    // nav_msgs::msg::Path path;
    // path.header.stamp = this->now();
    // path.header.frame_id = "map";
    // path.poses = section_path_[0];

    // path2_pub_->publish(path);

    timer_->cancel();
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_publisher::PathPublisher>());
    rclcpp::shutdown();
    return 0;
}