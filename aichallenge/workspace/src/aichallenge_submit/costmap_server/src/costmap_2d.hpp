#ifndef COSTMAP_SERVER_COSTMAP_2D_HPP_
#define COSTMAP_SERVER_COSTMAP_2D_HPP_

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace costmap_server
{
class CostMap2D : public rclcpp::Node
{

public:
    CostMap2D();

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_2d_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void send_map_request();
    void createInflationLayer();
    void createInflationLayer(
        std::vector<int8_t> & obstacle_map, nav_msgs::msg::OccupancyGrid & costmap
    );
    void calculateInflation(
        nav_msgs::msg::OccupancyGrid & map, const uint32_t & map_x, const uint32_t & map_y);
    void calculateInflation(const uint32_t & map_x, const uint32_t & map_y);
    double calculateCost(double stochastic_variable, double inflation_radius);
    double normalizeCost(double max_cost, double cost);
    std::tuple<int, int, int, int> calculateIndex(
        double x_m, double y_m, double radius
    );

    nav_msgs::msg::OccupancyGrid map_;
    bool get_map_;
    double width_, height_;
    double origin_x_, origin_y_;
    double resolution_;

    int outside_cost_;
    double wall_inflation_radius_;
    int wall_min_cost_, wall_max_cost_;

    int object_inside_cost_;
    double object_inside_radius_;
    double object_inflation_radius_;
    int object_min_cost_, object_max_cost_;

    int sample_;
    int aa_, bb_;
};
}

#endif