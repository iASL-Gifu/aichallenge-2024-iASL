#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"

class RouteNode : public rclcpp::Node
{
public:
    RouteNode()
    : Node("route_publisher_node")
    {
        // パラメータの宣言（デフォルト値を設定）
        this->declare_parameter("left_harf_pose.position.x", 0.0);
        this->declare_parameter("left_harf_pose.position.y", 0.0);
        this->declare_parameter("left_harf_pose.orientation.z", 0.0);
        this->declare_parameter("left_harf_pose.orientation.w", 0.0);

        this->declare_parameter("center_harf_pose.position.x", 0.0);
        this->declare_parameter("center_harf_pose.position.y", 0.0);
        this->declare_parameter("center_harf_pose.orientation.z", 0.0);
        this->declare_parameter("center_harf_pose.orientation.w", 0.0);

        this->declare_parameter("right_harf_pose.position.x", 0.0);
        this->declare_parameter("right_harf_pose.position.y", 0.0);
        this->declare_parameter("right_harf_pose.orientation.z", 0.0);
        this->declare_parameter("right_harf_pose.orientation.w", 0.0);

        this->declare_parameter("goal_pose.position.x", 0.0);
        this->declare_parameter("goal_pose.position.y", 0.0);
        this->declare_parameter("goal_pose.orientation.z", 0.0);
        this->declare_parameter("goal_pose.orientation.w", 0.0);

        this->declare_parameter("proximity_threshold", 1.0);
        this->declare_parameter("delay_route_publish_ms", 500);

        // YAMLファイルからのパラメータの取得
        this->get_parameter("left_harf_pose.position.x", left_harf_pose_x_);
        this->get_parameter("left_harf_pose.position.y", left_harf_pose_y_);
        this->get_parameter("left_harf_pose.orientation.z", left_harf_orientation_z_);
        this->get_parameter("left_harf_pose.orientation.w", left_harf_orientation_w_);

        this->get_parameter("center_harf_pose.position.x", center_harf_pose_x_);
        this->get_parameter("center_harf_pose.position.y", center_harf_pose_y_);
        this->get_parameter("center_harf_pose.orientation.z", center_harf_orientation_z_);
        this->get_parameter("center_harf_pose.orientation.w", center_harf_orientation_w_);

        this->get_parameter("right_harf_pose.position.x", right_harf_pose_x_);
        this->get_parameter("right_harf_pose.position.y", right_harf_pose_y_);
        this->get_parameter("right_harf_pose.orientation.z", right_harf_orientation_z_);
        this->get_parameter("right_harf_pose.orientation.w", right_harf_orientation_w_);

        this->get_parameter("goal_pose.position.x", goal_pose_x_);
        this->get_parameter("goal_pose.position.y", goal_pose_y_);
        this->get_parameter("goal_pose.orientation.z", goal_orientation_z_);
        this->get_parameter("goal_pose.orientation.w", goal_orientation_w_);

        this->get_parameter("right_harf_pose.position.x", target_pose_x_);
        this->get_parameter("right_harf_pose.position.y", target_pose_y_);

        this->get_parameter("proximity_threshold", target_pose_r_);
        this->get_parameter("delay_route_publish_ms", delay_time);

        RCLCPP_INFO(this->get_logger(), "Left Harf Pose: x=%f, y=%f, z=%f, w=%f", left_harf_pose_x_, left_harf_pose_y_, left_harf_orientation_z_, left_harf_orientation_w_);
        RCLCPP_INFO(this->get_logger(), "Center Harf Pose: x=%f, y=%f, z=%f, w=%f", center_harf_pose_x_, center_harf_pose_y_, center_harf_orientation_z_, center_harf_orientation_w_);
        RCLCPP_INFO(this->get_logger(), "Right Harf Pose: x=%f, y=%f, z=%f, w=%f", right_harf_pose_x_, right_harf_pose_y_, right_harf_orientation_z_, right_harf_orientation_w_);
        RCLCPP_INFO(this->get_logger(), "Goal Pose: x=%f, y=%f, z=%f, w=%f", goal_pose_x_, goal_pose_y_, goal_orientation_z_, goal_orientation_w_);
        RCLCPP_INFO(this->get_logger(), "Target Pose: x=%f, y=%f", target_pose_x_, target_pose_y_);
        RCLCPP_INFO(this->get_logger(), "Proximity Threshold: %f, Delay Time: %d", target_pose_r_, delay_time);


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

        // PredictedObjectsをSubするサブスクライバの作成
        predicted_objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
            "~/input/objects", 10,
            [this](const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
                this->predicted_objects_ = msg;
            });
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

        double distance_to_goal = std::hypot(current_x - target_pose_x_, current_y - target_pose_y_);

        // 距離が閾値より小さい場合にトリガーを引く
        if (distance_to_goal < target_pose_r_)
        {
            publish_route();
        }
    }

    void publish_route()
    {
        auto route_msg = autoware_planning_msgs::msg::LaneletRoute();
        route_msg.header.frame_id = "map";

        // 障害物の情報が保存されているか確認
        if (predicted_objects_)
        {
            // 各ハーフゴールポーズの近くに障害物があるかを確認
            bool left_harf_blocked = check_for_obstacle_near_pose(left_harf_pose_x_, left_harf_pose_y_, predicted_objects_);
            bool center_harf_blocked = check_for_obstacle_near_pose(center_harf_pose_x_, center_harf_pose_y_, predicted_objects_);
            bool right_harf_blocked = check_for_obstacle_near_pose(right_harf_pose_x_, right_harf_pose_y_, predicted_objects_);

            // セクション3, 4, 5の場合、障害物を避けたハーフポーズをゴールに設定
            if (current_section_ == 3)
            {
                if (!center_harf_blocked)
                {
                    harf_pose_x_ = center_harf_pose_x_;
                    harf_pose_y_ = center_harf_pose_y_;
                    harf_orientation_z_ = center_harf_orientation_z_;
                    harf_orientation_w_ = center_harf_orientation_w_;
                    RCLCPP_INFO(this->get_logger(), "Setting center harf_pose to Center Harf Pose: x=%f, y=%f, z=%f, w=%f", harf_pose_x_, harf_pose_y_, harf_orientation_z_, harf_orientation_w_);

                }
                else if (!left_harf_blocked)
                {
                    harf_pose_x_ = left_harf_pose_x_;
                    harf_pose_y_ = left_harf_pose_y_;
                    harf_orientation_z_ = left_harf_orientation_z_;
                    harf_orientation_w_ = left_harf_orientation_w_;
                    RCLCPP_INFO(this->get_logger(), "Setting left harf_pose to Center Harf Pose: x=%f, y=%f, z=%f, w=%f", harf_pose_x_, harf_pose_y_, harf_orientation_z_, harf_orientation_w_);

                }
                else if (!right_harf_blocked)
                {
                    harf_pose_x_ = right_harf_pose_x_;
                    harf_pose_y_ = right_harf_pose_y_;
                    harf_orientation_z_ = right_harf_orientation_z_;
                    harf_orientation_w_ = right_harf_orientation_w_;
                    RCLCPP_INFO(this->get_logger(), "Setting right harf_pose to Center Harf Pose: x=%f, y=%f, z=%f, w=%f", harf_pose_x_, harf_pose_y_, harf_orientation_z_, harf_orientation_w_);

                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "All harf poses are blocked by obstacles.");
                }

            }
            
        }

        if (current_section_ == 3)
        {
            // スタートをGoalに、ゴールを選ばれたハーフポーズに設定
            route_msg.start_pose.position.x = goal_pose_x_;
            route_msg.start_pose.position.y = goal_pose_y_;
            route_msg.start_pose.orientation.z = goal_orientation_z_;
            route_msg.start_pose.orientation.w = goal_orientation_w_;

            route_msg.goal_pose.position.x = harf_pose_x_;
            route_msg.goal_pose.position.y = harf_pose_y_;
            route_msg.goal_pose.orientation.z = harf_orientation_z_;
            route_msg.goal_pose.orientation.w = harf_orientation_w_;

            target_pose_x_ = goal_pose_x_;
            target_pose_y_ = goal_pose_y_;

            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }
        else if (current_section_ == 7)
        {
            // セクション7の場合、スタートをハーフポーズ、ゴールをGoalに設定
            route_msg.start_pose.position.x = harf_pose_x_;
            route_msg.start_pose.position.y = harf_pose_y_;
            route_msg.start_pose.orientation.z = harf_orientation_z_;
            route_msg.start_pose.orientation.w = harf_orientation_w_;

            route_msg.goal_pose.position.x = goal_pose_x_;
            route_msg.goal_pose.position.y = goal_pose_y_;
            route_msg.goal_pose.orientation.z = goal_orientation_z_;
            route_msg.goal_pose.orientation.w = goal_orientation_w_;

            target_pose_x_ = harf_pose_x_;
            target_pose_y_ = harf_pose_y_;
        }

        route_pub_->publish(route_msg);
    }

    bool check_for_obstacle_near_pose(double pose_x, double pose_y, const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr& msg)
    {
        // 障害物が指定したポーズの近くにあるかを確認
        for (const auto &object : msg->objects)
        {
            double object_x = object.kinematics.initial_pose_with_covariance.pose.position.x;
            double object_y = object.kinematics.initial_pose_with_covariance.pose.position.y;

            double distance = std::hypot(pose_x - object_x, pose_y - object_y);
            if (distance < target_pose_r_)
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected near pose: (%f, %f)", pose_x, pose_y);
                return true;
            }
        }
        return false;
    }

    // メンバ変数
    double left_harf_pose_x_, left_harf_pose_y_, left_harf_orientation_z_, left_harf_orientation_w_;
    double center_harf_pose_x_, center_harf_pose_y_, center_harf_orientation_z_, center_harf_orientation_w_;
    double right_harf_pose_x_, right_harf_pose_y_, right_harf_orientation_z_, right_harf_orientation_w_;
    double goal_pose_x_, goal_pose_y_, goal_orientation_z_, goal_orientation_w_;
    double harf_pose_x_, harf_pose_y_, harf_orientation_z_, harf_orientation_w_;
    double target_pose_x_, target_pose_y_, target_pose_r_;
    int delay_time;
    int current_section_ = 0;
    nav_msgs::msg::Odometry::SharedPtr odometry_;

    autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr predicted_objects_; // オブジェクト情報を保持

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr object_change_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematics_;
    rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_objects_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RouteNode>());
    rclcpp::shutdown();
    return 0;
}
