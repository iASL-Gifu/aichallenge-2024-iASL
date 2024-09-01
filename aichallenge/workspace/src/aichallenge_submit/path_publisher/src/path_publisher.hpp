#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_service/srv/get_path.hpp"
#include "path_service/srv/get_obstacle_path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>

namespace path_publisher
{

class PathPublisher : public rclcpp::Node
{

public:
    PathPublisher();

private:
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path2_pub_;

    // Subscription
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    std::vector<std::pair<double, double>> objects_coordinate_;
    int current_section_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    nav_msgs::msg::Odometry odometry_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    void object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Client
    std::vector<rclcpp::Client<path_service::srv::GetObstaclePath>::SharedPtr> get_obstacle_path_clients_;

    // Server
    rclcpp::Service<path_service::srv::GetPath>::SharedPtr get_path_srv_;
    void handle_get_path(const std::shared_ptr<path_service::srv::GetPath::Request> request,
                                std::shared_ptr<path_service::srv::GetPath::Response> response);

    // Param
    std::string raceline_csv_path_;
    std::string centerline_csv_path_;
    int centerline_downsample_rate_, raceline_downsample_rate_;
    int publish_rate_;
    int margin_;
    std::vector<double> section_point_;
    int start_index_;

    // function
    void load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point);
    void divide_section();

    // variable
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> raceline_points_;
    std::vector<geometry_msgs::msg::PoseStamped> centerline_points_;
    std::vector<geometry_msgs::msg::PoseStamped> optimize_points_;
    std::vector<std::vector<geometry_msgs::msg::PoseStamped>> section_path_;
};

}

#endif