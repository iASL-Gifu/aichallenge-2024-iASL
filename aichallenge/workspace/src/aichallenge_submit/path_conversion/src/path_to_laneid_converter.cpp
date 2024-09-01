#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "nav_msgs/msg/path.hpp"

class PathToLaneidConverter : public rclcpp::Node
{
public:
    using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
    using PathPointWithLaneId = autoware_auto_planning_msgs::msg::PathPointWithLaneId;

    PathToLaneidConverter() : Node("path_to_laneid_converter")
    {
        RCLCPP_INFO(this->get_logger(), "============= Path Conversion ==============");
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "obstacle_avoidance", 1, 
            std::bind(&PathToLaneidConverter::path_callback, this, std::placeholders::_1)
        );

        path_pub_ = this->create_publisher<PathWithLaneId>("obstacle_path_with_laneid", 1);
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        PathWithLaneId new_path = PathWithLaneId();
        new_path.header = msg->header;

        for (const auto & point : msg->poses) {
            PathPointWithLaneId next_point = PathPointWithLaneId();
            next_point.point.pose.position.x = point.pose.position.x;
            next_point.point.pose.position.y = point.pose.position.y;
            next_point.point.pose.position.z = 43.1;

            next_point.point.pose.orientation.x = point.pose.orientation.x;
            next_point.point.pose.orientation.y = point.pose.orientation.y;
            next_point.point.pose.orientation.z = point.pose.orientation.z;
            next_point.point.pose.orientation.w = point.pose.orientation.w;

            next_point.point.longitudinal_velocity_mps = 100.0;
            next_point.point.lateral_velocity_mps = 0.0;
            next_point.point.heading_rate_rps = 0.0;
            next_point.point.is_final = false;

            new_path.points.push_back(next_point);
        }

        path_pub_->publish(new_path);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<PathWithLaneId>::SharedPtr path_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathToLaneidConverter>());
    rclcpp::shutdown();
    return 0;
}