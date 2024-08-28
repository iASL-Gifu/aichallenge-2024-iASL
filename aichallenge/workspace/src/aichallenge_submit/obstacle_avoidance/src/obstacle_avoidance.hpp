#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"


namespace obstacle_avoidance
{

class ObstacleAvoidance : public rclcpp::Node
{

public:
    using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
    using PathPointWithLaneId = autoware_auto_planning_msgs::msg::PathPointWithLaneId;

    ObstacleAvoidance();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<PathWithLaneId>::SharedPtr path_sub_;
    rclcpp::Publisher<PathWithLaneId>::SharedPtr avoidance_path_pub_;

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const PathWithLaneId::SharedPtr msg);

    // double compute_attractive_potential(
    //     geometry_msgs::msg::Pose &start_pose, geometry_msgs::msg::Pose &end_pose
    // );
    // double ObstacleAvoidance::compute_repulsive_potential(
    //     geometry_msgs::msg::Pose &start_pose,
    //     const std::vector<double> &ox, const std::vector<double> &oy
    // );
    double compute_attractive_potential(double x, double y, double gx, double gy);
    double compute_repulsive_potential(
        double x, double y, const std::vector<double>& ox, const std::vector<double>& oy
    );
    void euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result);

    double threshold_;
    double attract_;
    double repulse_;
    double penalty_dist_;
    double desired_dist_;
    int start_index_;
    int num_change_points_;
    double lookahead_dist_;
    double angle_interval_;
    double max_angle_;
    double min_angle_;
    double near_point_dist_;

    nav_msgs::msg::OccupancyGrid costmap_;
    double width_, height_, resolution_, origin_x_, origin_y_;
    nav_msgs::msg::Odometry odometry_;
    PathWithLaneId path_;

    bool costmap_received_, path_received_;
    std::vector<double> angles;
    std::vector<PathPointWithLaneId> generate_paths_;
};

}


#endif