#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"

class RouteNode : public rclcpp::Node
{
public:
    RouteNode()
    : Node("route_publisher_node")
    {
        // パラメータの宣言（デフォルト値を設定）
        this->declare_parameter("harf_pose.position.x", 0.0);
        this->declare_parameter("harf_pose.position.y", 0.0);
        this->declare_parameter("pit_stop_pose.position.x", 0.0);
        this->declare_parameter("pit_stop_pose.position.y", 0.0);

        this->declare_parameter("harf_pose.orientation.z", 0.0);
        this->declare_parameter("harf_pose.orientation.w", 0.0);
        this->declare_parameter("pit_stop_pose.orientation.z", 0.0);
        this->declare_parameter("pit_stop_pose.orientation.w", 0.0);

        // YAMLファイルからのパラメータの取得
        this->get_parameter("harf_pose.position.x", harf_pose_x_);
        this->get_parameter("harf_pose.position.y", harf_pose_y_);
        this->get_parameter("pit_stop_pose.position.x", pit_pose_x_);
        this->get_parameter("pit_stop_pose.position.y", pit_pose_y_);

        this->get_parameter("harf_pose.orientation.z", harf_orientation_z_);
        this->get_parameter("harf_pose.orientation.w", harf_orientation_w_);
        this->get_parameter("pit_stop_pose.orientation.z", pit_orientation_z_);
        this->get_parameter("pit_stop_pose.orientation.w", pit_orientation_w_);

        // サブスクリプションの作成
        section_change_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "~/input/section_change", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->handle_section_change(msg);
            });

        // Publisherの作成
        route_pub_ = this->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
            "~/output/route", rclcpp::QoS{1}.transient_local());
    }

private:
    void handle_section_change(const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto route_msg = autoware_planning_msgs::msg::LaneletRoute();

        if (msg->data == 45)
        {
            route_msg.header.frame_id = "map";
            route_msg.start_pose.position.x = pit_pose_x_;
            route_msg.start_pose.position.y = pit_pose_y_;
            route_msg.start_pose.orientation.z = pit_orientation_z_;
            route_msg.start_pose.orientation.w = pit_orientation_w_;
            
            route_msg.goal_pose.position.x = harf_pose_x_;
            route_msg.goal_pose.position.y = harf_pose_y_;
            route_msg.goal_pose.orientation.z = harf_orientation_z_;
            route_msg.goal_pose.orientation.w = harf_orientation_w_;
        }
        else if (msg->data == 81)
        {
            route_msg.header.frame_id = "map";

            route_msg.start_pose.position.x = harf_pose_x_;
            route_msg.start_pose.position.y = harf_pose_y_;
            route_msg.start_pose.orientation.z = harf_orientation_z_;
            route_msg.start_pose.orientation.w = harf_orientation_w_;

            route_msg.goal_pose.position.x = pit_pose_x_;
            route_msg.goal_pose.position.y = pit_pose_y_;
            route_msg.goal_pose.orientation.z = pit_orientation_z_;
            route_msg.goal_pose.orientation.w = pit_orientation_w_;
        }

        route_pub_->publish(route_msg);
    }

    // メンバ変数
    double harf_pose_x_, harf_pose_y_, pit_pose_x_, pit_pose_y_;
    double harf_orientation_z_, harf_orientation_w_, pit_orientation_z_, pit_orientation_w_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr section_change_sub_;
    rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RouteNode>());
    rclcpp::shutdown();
    return 0;
}
