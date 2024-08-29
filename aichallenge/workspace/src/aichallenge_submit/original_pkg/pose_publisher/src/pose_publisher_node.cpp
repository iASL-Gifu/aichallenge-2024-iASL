#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <random>
#include <cmath> // 追加

class RoutePlannerNode : public rclcpp::Node
{
public:
  RoutePlannerNode()
  : Node("route_planner_node")
  {
    // Lookaheadパラメータの宣言とデフォルト値の設定
    this->declare_parameter<int>("lookahead_index", 1);
    
    // Publish Hzのパラメータの宣言とデフォルト値の設定
    this->declare_parameter<double>("publish_hz", 1.0); // デフォルトは1Hz

    // Safe Distanceのパラメータの宣言とデフォルト値の設定
    this->declare_parameter<double>("safe_distance", 0.0); // デフォルトは1.0メートル

    // Publisherを作成
    route_pub_ = this->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
    "~/output/route", rclcpp::QoS{1}.transient_local());


    // OdometryとTrajectoryのSubscriberを作成
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", 1, std::bind(&RoutePlannerNode::odom_callback, this, std::placeholders::_1));
    
    traj_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", 1, std::bind(&RoutePlannerNode::trajectory_callback, this, std::placeholders::_1));

    // 障害物データのSubscriberを作成
    object_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/input/objects", 1,
      std::bind(&RoutePlannerNode::data_callback, this, std::placeholders::_1));

    // タイマーを作成し、パブリッシュ頻度を設定
    double publish_hz = 1.0;
    this->get_parameter("publish_hz", publish_hz);
    auto publish_interval = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = this->create_wall_timer(publish_interval, std::bind(&RoutePlannerNode::publish_route, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    start_pose_ = msg->pose.pose;  // スタートポーズを更新
  }

  void trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
      return;
    }

    // パラメータからLookahead先のインデックスを取得
    this->get_parameter("lookahead_index", lookahead_index_);

    // Lookahead先のインデックスの点をゴールポーズとする
    auto goal_index = std::min(static_cast<size_t>(lookahead_index_), msg->points.size() - 1);
    goal_pose_ = msg->points[goal_index].pose;
  }

  void data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
      if (msg->data.empty()) {
          RCLCPP_WARN(this->get_logger(), "Received empty objects data");
          return;
      }

      double safe_distance;
      this->get_parameter("safe_distance", safe_distance);

      bool goal_is_safe = false;
      while (!goal_is_safe && lookahead_index_ < msg->data.size() / 3) { 
          goal_is_safe = true;
          for (size_t i = 0; i < msg->data.size(); i += 3) {
              double x = msg->data[i];
              double y = msg->data[i+1];
              double radius = msg->data[i+2];

              // 障害物との距離を計算
              double distance = std::sqrt(std::pow(goal_pose_.position.x - x, 2) +
                                          std::pow(goal_pose_.position.y - y, 2));

              // ゴールポーズと障害物が近すぎる場合、インデックスをインクリメント
              if (distance < radius + safe_distance) {
                  lookahead_index_++;
                  RCLCPP_INFO(this->get_logger(), "Obstacle detected near goal, increasing lookahead index to %d", lookahead_index_);
                  goal_is_safe = false;
                  break; // 安全なゴールを見つけるためにループを抜ける
              }
          }
      }

      if (!goal_is_safe) {
          RCLCPP_WARN(this->get_logger(), "No safe goal found, using last available point as goal");
          lookahead_index_ = msg->data.size() / 3 - 1; // 最後のポイントをゴールとして使用
      }
  }


  void publish_route()
  {
    // スタートポーズとゴールポーズが未設定の場合は何もしない
    if (start_pose_ == geometry_msgs::msg::Pose() || goal_pose_ == geometry_msgs::msg::Pose()) {
      RCLCPP_WARN(this->get_logger(), "Start or goal pose is not set");
      return;
    }

    auto message = autoware_planning_msgs::msg::LaneletRoute();
    
    // Headerの設定
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // スタートポーズとゴールポーズの設定
    message.start_pose = start_pose_;
    message.goal_pose = goal_pose_;

    // UUIDの設定
    message.uuid = generate_uuid();

    // セグメントの設定
    autoware_planning_msgs::msg::LaneletSegment segment1;
    autoware_planning_msgs::msg::LaneletPrimitive primitive1;
    primitive1.id = 705;
    primitive1.primitive_type = "";  // 空のprimitive_typeを設定

    segment1.preferred_primitive = primitive1;
    segment1.primitives.push_back(primitive1);

    autoware_planning_msgs::msg::LaneletSegment segment2;
    autoware_planning_msgs::msg::LaneletPrimitive primitive2;
    primitive2.id = 704;
    primitive2.primitive_type = "";  // 空のprimitive_typeを設定

    segment2.preferred_primitive = primitive2;
    segment2.primitives.push_back(primitive2);

    message.segments.push_back(segment1);
    message.segments.push_back(segment2);

    // メッセージをPublish
    route_pub_->publish(message);
  }

  unique_identifier_msgs::msg::UUID generate_uuid()
  {
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(0, 255);

    for (auto &byte : uuid_msg.uuid) {
      byte = dis(gen);
    }

    return uuid_msg;
  }

  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr object_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  
  int lookahead_index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoutePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
