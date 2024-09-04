#include "rclcpp/rclcpp.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "visualization_msgs/msg/marker.hpp"

class StartGoalMarkerNode : public rclcpp::Node
{
public:
    StartGoalMarkerNode()
    : Node("start_goal_marker_node")
    {
        // Routeトピックをサブスクライブ
        route_sub_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
            "~/input/route", 10,
            std::bind(&StartGoalMarkerNode::route_callback, this, std::placeholders::_1));

        // Markerトピックをパブリッシュ (QoSを指定)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "~/output/goal_marker", rclcpp::QoS(10));
    }

private:
    void route_callback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr route_msg)
    {
        // StartのMarker (緑色)
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = this->now();
        start_marker.ns = "start";
        start_marker.id = 0;  // StartのMarker ID
        start_marker.type = visualization_msgs::msg::Marker::ARROW;  // 矢印
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.scale.x = 1.0;  // 矢印の長さ
        start_marker.scale.y = 0.2;  // 矢印の太さ
        start_marker.scale.z = 0.2;  // 矢印の高さ
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;  // 緑色
        start_marker.pose.position.x = route_msg->start_pose.position.x;
        start_marker.pose.position.y = route_msg->start_pose.position.y;
        start_marker.pose.position.z = route_msg->start_pose.position.z;
        start_marker.pose.orientation.x = route_msg->start_pose.orientation.x;
        start_marker.pose.orientation.y = route_msg->start_pose.orientation.y;
        start_marker.pose.orientation.z = route_msg->start_pose.orientation.z;
        start_marker.pose.orientation.w = route_msg->start_pose.orientation.w;

        // GoalのMarker (赤色)
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = this->now();
        goal_marker.ns = "goal";
        goal_marker.id = 1;  // GoalのMarker ID
        goal_marker.type = visualization_msgs::msg::Marker::ARROW;  // 矢印
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.scale.x = 1.0;  // 矢印の長さ
        goal_marker.scale.y = 0.2;  // 矢印の太さ
        goal_marker.scale.z = 0.2;  // 矢印の高さ
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        goal_marker.color.a = 1.0;  // 赤色
        goal_marker.pose.position.x = route_msg->goal_pose.position.x;
        goal_marker.pose.position.y = route_msg->goal_pose.position.y;
        goal_marker.pose.position.z = route_msg->goal_pose.position.z;
        goal_marker.pose.orientation.x = route_msg->goal_pose.orientation.x;
        goal_marker.pose.orientation.y = route_msg->goal_pose.orientation.y;
        goal_marker.pose.orientation.z = route_msg->goal_pose.orientation.z;
        goal_marker.pose.orientation.w = route_msg->goal_pose.orientation.w;

        // Markerをパブリッシュ
        marker_pub_->publish(start_marker);
        marker_pub_->publish(goal_marker);
    }

    rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartGoalMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
