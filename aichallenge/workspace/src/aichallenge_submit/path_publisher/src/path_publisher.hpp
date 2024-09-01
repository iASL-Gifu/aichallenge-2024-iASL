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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Service<path_service::srv::GetPath>::SharedPtr get_path_srv_;
    std::vector<rclcpp::Client<path_service::srv::GetObstaclePath>::SharedPtr> get_obstacle_path_clients_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handle_get_path(const std::shared_ptr<path_service::srv::GetPath::Request> request,
                                    std::shared_ptr<path_service::srv::GetPath::Response> response);


    nav_msgs::msg::Odometry odometry_;

    std::string raceline_csv_path_;
    std::string centerline_csv_path_;
    int centerline_downsample_rate_, raceline_downsample_rate_;
    int loop_count_;
    int publish_rate_;
    int margin_;

    std::vector<geometry_msgs::msg::PoseStamped> raceline_points_;
    std::vector<geometry_msgs::msg::PoseStamped> centerline_points_;
    // bool csv_loaded_;
    bool path_initialized_;
    // bool odometry_received_;
    int current_index_; 


    int line_count_;
    int current_section_;

    void load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point);

    double prev_x_, prev_y_;
};

}

#endif