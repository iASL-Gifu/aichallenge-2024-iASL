#ifndef CREATE_ROUTE_HPP_
#define CREATE_ROUTE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_service/srv/get_path.hpp"
#include "path_service/srv/get_obstacle_path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


namespace create_route
{

class Create_route : public rclcpp::Node
{

public:
    using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
    using PathPointWithLaneId = autoware_auto_planning_msgs::msg::PathPointWithLaneId;

    Create_route();

private:
    // Publish
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr marker_path_pub_;
    

    // Subscription
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid costmap_;
    double width_, height_, resolution_, origin_x_, origin_y_;
    bool costmap_received_;
    
    

    // Client
    rclcpp::Client<path_service::srv::GetPath>::SharedPtr get_path_client_;
    rclcpp::TimerBase::SharedPtr path_timer_;
    void send_path_request();
    nav_msgs::msg::Path section_path_;
    nav_msgs::msg::Path center_path_;
    bool get_path_;
    bool get_center_path_;

    // Service
    rclcpp::Service<path_service::srv::GetObstaclePath>::SharedPtr get_obstacle_path_srv_;
    void handle_get_obstacle_path(const std::shared_ptr<path_service::srv::GetObstaclePath::Request> request,
                                    std::shared_ptr<path_service::srv::GetObstaclePath::Response> response);
    nav_msgs::msg::Path avoidance_path_;

    // param
    int start_index_, end_index_;
    int section_count_;
    std::string left_csv_path_, right_csv_path_;
    double margin_radius_;
    double angle_interval_;
    double collision_checker_;

    // function
    int find_nearest_point_index(double x,double y);
    std::vector<int> get_indices(int index, int start, int end);
    void euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result);
    void load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point);

    // variable
    // first: 進む距離, second: 角度
    std::vector<std::pair<double, double>> generate_path_;
    std::vector<geometry_msgs::msg::PoseStamped> left_path_;
    std::vector<geometry_msgs::msg::PoseStamped> right_path_;
    rclcpp::TimerBase::SharedPtr delayed_timer_;

};

}


#endif