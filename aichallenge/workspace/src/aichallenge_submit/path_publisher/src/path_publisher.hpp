#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    nav_msgs::msg::Odometry odometry_;

    std::string csv_path_;
    int downsample_rate_;
    int loop_count_;
    int publish_rate_;
    int margin_;

    std::vector<geometry_msgs::msg::PoseStamped> points_;
    bool csv_loaded_;
    bool path_initialized_;
    bool odometry_received_;
    int current_index_; 


    int line_count_;
};

}

#endif