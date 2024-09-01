#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "nav_msgs/msg/odometry.hpp" // 追加

class RouteNode : public rclcpp::Node
{
public:
    RouteNode()
    : Node("route_publisher_node")
    {
        // パラメータの宣言（デフォルト値を設定）
        this->declare_parameter("harf_pose.position.x", 0.0);
        this->declare_parameter("harf_pose.position.y", 0.0);
        this->declare_parameter("goal_pose.position.x", 0.0);
        this->declare_parameter("goal_pose.position.y", 0.0);

        this->declare_parameter("harf_pose.orientation.z", 0.0);
        this->declare_parameter("harf_pose.orientation.w", 0.0);
        this->declare_parameter("goal_pose.orientation.z", 0.0);
        this->declare_parameter("goal_pose.orientation.w", 0.0);

        this->declare_parameter("target_threthold", 2.0);

        // YAMLファイルからのパラメータの取得
        this->get_parameter("harf_pose.position.x", harf_pose_x_);
        this->get_parameter("harf_pose.position.y", harf_pose_y_);
        this->get_parameter("goal_pose.position.x", goal_pose_x_);
        this->get_parameter("goal_pose.position.y", goal_pose_y_);

        this->get_parameter("harf_pose.orientation.z", harf_orientation_z_);
        this->get_parameter("harf_pose.orientation.w", harf_orientation_w_);
        this->get_parameter("goal_pose.orientation.z", goal_orientation_z_);
        this->get_parameter("goal_pose.orientation.w", goal_orientation_w_);

        this->get_parameter("target_threthold", proximity_threshold_);

        // サブスクリプションの作成
        object_change_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "~/input/object_event", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->handle_section_change(msg);
            });

        // Odometryサブスクリプションの作成
        sub_kinematics_ = create_subscription<nav_msgs::msg::Odometry>(
            "~/input/kinematics", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
                odometry_ = msg; 
                check_proximity_and_publish(); 
            });

        // Publisherの作成
        route_pub_ = this->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
            "~/output/route", rclcpp::QoS{1}.transient_local());
    }

private:
    void handle_section_change(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_section_ = msg->data;
    }

    void check_proximity_and_publish()
    {
        if (!odometry_) return;

        // 現在位置を取得
        double current_x = odometry_->pose.pose.position.x;
        double current_y = odometry_->pose.pose.position.y;

        // 目標地点との距離を計算
        double distance_to_goal = std::hypot(current_x - target_pose_x, current_y - target_pose_y);

        // 距離が閾値より小さい場合にトリガーを引く
        if (distance_to_goal < proximity_threshold_)
        {
            publish_route();
        }
        // RCLCPP_INFO(this->get_logger(), "target near");
    }

    void publish_route()
    {
        auto route_msg = autoware_planning_msgs::msg::LaneletRoute();
        route_msg.header.frame_id = "map";

        if (current_section_ == 3)
        {
            route_msg.start_pose.position.x = goal_pose_x_;
            route_msg.start_pose.position.y = goal_pose_y_;
            route_msg.start_pose.orientation.z = goal_orientation_z_;
            route_msg.start_pose.orientation.w = goal_orientation_w_;
            
            route_msg.goal_pose.position.x = harf_pose_x_;
            route_msg.goal_pose.position.y = harf_pose_y_;
            route_msg.goal_pose.orientation.z = harf_orientation_z_;
            route_msg.goal_pose.orientation.w = harf_orientation_w_;

            target_pose_x = goal_pose_x_;
            target_pose_y = goal_pose_y_;

            RCLCPP_INFO(this->get_logger(), "target pit pose");
        }
        else if (current_section_ == 7)
        {
            route_msg.start_pose.position.x = harf_pose_x_;
            route_msg.start_pose.position.y = harf_pose_y_;
            route_msg.start_pose.orientation.z = harf_orientation_z_;
            route_msg.start_pose.orientation.w = harf_orientation_w_;

            route_msg.goal_pose.position.x = goal_pose_x_;
            route_msg.goal_pose.position.y = goal_pose_y_;
            route_msg.goal_pose.orientation.z = goal_orientation_z_;
            route_msg.goal_pose.orientation.w = goal_orientation_w_;

            target_pose_x = harf_pose_x_;
            target_pose_y = harf_pose_y_;

            RCLCPP_INFO(this->get_logger(), "target harf pose");
        }

        route_pub_->publish(route_msg);
    }

    // メンバ変数
    double harf_pose_x_, harf_pose_y_, goal_pose_x_, goal_pose_y_;
    double harf_orientation_z_, harf_orientation_w_, goal_orientation_z_, goal_orientation_w_;
    double target_pose_x = 89657.326, target_pose_y = 43176.6794;
    double proximity_threshold_ = 5.0; // 目標地点に近づいたと判断する距離の閾値（メートル）
    int current_section_ = 0;
    nav_msgs::msg::Odometry::SharedPtr odometry_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr object_change_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematics_;
    rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RouteNode>());
    rclcpp::shutdown();
    return 0;
}
