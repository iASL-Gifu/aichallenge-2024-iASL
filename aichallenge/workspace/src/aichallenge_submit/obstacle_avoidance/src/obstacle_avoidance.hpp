#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_service/srv/get_path.hpp"


namespace obstacle_avoidance
{

class ObstacleAvoidance : public rclcpp::Node
{

public:
    using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
    using PathPointWithLaneId = autoware_auto_planning_msgs::msg::PathPointWithLaneId;

    ObstacleAvoidance();

private:
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr avoidance_path_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_2d_pub_;

    // Subscription
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid costmap_;
    double width_, height_, resolution_, origin_x_, origin_y_;
    bool costmap_received_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    void path_callback(nav_msgs::msg::Path::SharedPtr msg);
    nav_msgs::msg::Path path_;
    bool path_received_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    nav_msgs::msg::Odometry odometry_;

    // Client
    rclcpp::Client<path_service::srv::GetPath>::SharedPtr get_path_client_;
    void send_path_request();
    rclcpp::TimerBase::SharedPtr path_timer_;
    nav_msgs::msg::Path centerline_path_;
    bool get_path_centerline_;

    // Server

    // param
    double attract_, repulse_;
    double forward_dist_, side_dist_;

    double desired_dist_;
    int start_index_;
    int num_change_points_;
    double lookahead_dist_;

    double angle_interval_;
    double max_angle_, min_angle_;
    double near_point_dist_;
    int margin_;

    bool visual_;

    // fuction
    bool are_positions_equal();
    double compute_attractive_potential(double x, double y, double gx, double gy);
    double compute_repulsive_potential(double x, double y, double angle_rad);
    std::pair<int, int> rotatePoint(double x, double y, double theta, double center_x, double center_y);
    void euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result);

    // variable
    float ex_x_, ex_y_;
    std::vector<geometry_msgs::msg::PoseStamped> generate_paths_;
    std::vector<double> angles;
    bool first_publish_;
};

}


#endif