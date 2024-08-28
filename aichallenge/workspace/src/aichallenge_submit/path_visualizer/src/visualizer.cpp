#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathVisualizer : public rclcpp::Node
{
public:
    using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;

    PathVisualizer() : Node("path_visualizer")
    {
        path_sub_ = this->create_subscription<PathWithLaneId>(
            "output",
            1, std::bind(&PathVisualizer::path_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_path", 1);
    }

private:
    void path_callback(const PathWithLaneId::SharedPtr msg)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header = msg->header;

        for (const auto & point : msg->points)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.pose = point.point.pose;
            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }

    rclcpp::Subscription<PathWithLaneId>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}
