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


        // サブスクリプションの作成
        object_change_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "~/input/object_event", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->handle_section_change(msg);
            });

        // PredictedObjectsをSubするサブスクライバの作成
        predicted_objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
            "~/input/objects", 10,
            [this](const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
                this->predicted_objects_ = msg;
            });

        // Publisherの作成
        route_pub_ = this->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
            "~/output/route", rclcpp::QoS{1}.transient_local());
    }

private:
    void handle_section_change(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_section_ = msg->data;

        // Objecteventが4の場合、障害物との距離を計算し、ハーフポーズを決定する
        if (current_section_ == 4)
        {
            determine_harf_pose_based_on_obstacles();
            publish_route();
        }
        // Objecteventが7の場合、直接ルートをパブリッシュする
        else if (current_section_ == 7)
        {
            publish_route();
        }
    }

    void determine_harf_pose_based_on_obstacles()
    {
        if (!predicted_objects_)
        {
            RCLCPP_WARN(this->get_logger(), "No predicted objects data available.");
            return;
        }

        // 各ハーフゴールポーズとの距離を比較
        double max_distance = -1;
        
        // 障害物との距離を比較し、一番遠いポーズを選ぶ
        double left_distance = calculate_min_distance_to_obstacles(left_harf_pose_x_, left_harf_pose_y_, predicted_objects_);
        double center_distance = calculate_min_distance_to_obstacles(center_harf_pose_x_, center_harf_pose_y_, predicted_objects_);
        double right_distance = calculate_min_distance_to_obstacles(right_harf_pose_x_, right_harf_pose_y_, predicted_objects_);


        if (left_distance > max_distance)
        {
            max_distance = left_distance;
            harf_pose_x_ = left_harf_pose_x_;
            harf_pose_y_ = left_harf_pose_y_;
            harf_orientation_z_ = left_harf_orientation_z_;
            harf_orientation_w_ = left_harf_orientation_w_;
            RCLCPP_INFO(this->get_logger(), "Left Harf Pose selected.");
        }

        if (center_distance > max_distance)
        {
            max_distance = center_distance;
            harf_pose_x_ = center_harf_pose_x_;
            harf_pose_y_ = center_harf_pose_y_;
            harf_orientation_z_ = center_harf_orientation_z_;
            harf_orientation_w_ = center_harf_orientation_w_;
            RCLCPP_INFO(this->get_logger(), "Center Harf Pose selected.");
        }

        if (right_distance > max_distance)
        {
            max_distance = right_distance;
            harf_pose_x_ = right_harf_pose_x_;
            harf_pose_y_ = right_harf_pose_y_;
            harf_orientation_z_ = right_harf_orientation_z_;
            harf_orientation_w_ = right_harf_orientation_w_;
            RCLCPP_INFO(this->get_logger(), "Right Harf Pose selected.");
        }
        
        RCLCPP_INFO(this->get_logger(), ": left=%f, center=%f, right=%f ", left_distance, center_distance, right_distance);
        
    }

    void publish_route()
    {
        auto route_msg = autoware_planning_msgs::msg::LaneletRoute();
        route_msg.header.frame_id = "map";

        if (current_section_ == 4)
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
        }
        else if (current_section_ == 7)
        {
            // スタートをハーフポーズ、ゴールをGoalに設定
            route_msg.start_pose.position.x = harf_pose_x_;
            route_msg.start_pose.position.y = harf_pose_y_;
            route_msg.start_pose.orientation.z = harf_orientation_z_;
            route_msg.start_pose.orientation.w = harf_orientation_w_;

            route_msg.goal_pose.position.x = goal_pose_x_;
            route_msg.goal_pose.position.y = goal_pose_y_;
            route_msg.goal_pose.orientation.z = goal_orientation_z_;
            route_msg.goal_pose.orientation.w = goal_orientation_w_;
        }

        route_pub_->publish(route_msg);
        RCLCPP_INFO(this->get_logger(), "Route published.");
    }

    double calculate_min_distance_to_obstacles(double pose_x, double pose_y, const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr& msg)
    {
        double min_distance = std::numeric_limits<double>::max();

        for (const auto &object : msg->objects)
        {
            double object_x = object.kinematics.initial_pose_with_covariance.pose.position.x;
            double object_y = object.kinematics.initial_pose_with_covariance.pose.position.y;

            double distance = std::hypot(pose_x - object_x, pose_y - object_y);
            if (distance < min_distance)
            {
                min_distance = distance;
            }
        }

        return min_distance;
    }

    // メンバ変数
    double left_harf_pose_x_, left_harf_pose_y_, left_harf_orientation_z_, left_harf_orientation_w_;
    double center_harf_pose_x_, center_harf_pose_y_, center_harf_orientation_z_, center_harf_orientation_w_;
    double right_harf_pose_x_, right_harf_pose_y_, right_harf_orientation_z_, right_harf_orientation_w_;
    double goal_pose_x_, goal_pose_y_, goal_orientation_z_, goal_orientation_w_;
    double harf_pose_x_, harf_pose_y_, harf_orientation_z_, harf_orientation_w_;

    int current_section_ = 0;
    autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr predicted_objects_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr object_change_sub_;
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
