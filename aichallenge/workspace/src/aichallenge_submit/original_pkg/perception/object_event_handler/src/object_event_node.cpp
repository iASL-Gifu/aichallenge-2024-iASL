#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "std_msgs/msg/int32.hpp"

class ObstacleUpdateNode : public rclcpp::Node
{
public:
  ObstacleUpdateNode() : Node("obstacle_update_node")
  {
    // サブスクライバを作成
    data_subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
        "~/input/objects", 10,
        std::bind(&ObstacleUpdateNode::data_callback, this, std::placeholders::_1));

    // パブリッシャを作成
    index_publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output/object_event", 10);

    // 前の時刻のデータを初期化
    prev_data_.resize(32); // 32個の障害物データを保存するためのベクタ
  }

private:
  void data_callback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    // 現在のデータを取得
    auto current_data = msg->objects;

    // 障害物データの更新を確認
    for (size_t i = 0; i < current_data.size(); ++i)
    {
      const auto& current_object = current_data[i];
      if (i >= prev_data_.size() || has_object_updated(prev_data_[i], current_object))
      {
        RCLCPP_INFO(this->get_logger(), "Obstacle %zu updated", i);

        // 更新された障害物のインデックスをPub
        std_msgs::msg::Int32 index_msg;
        index_msg.data = i;
        index_publisher_->publish(index_msg);
      }
    }

    // 現在のデータを保存して次のループに備える
    prev_data_ = current_data;
  }

  bool has_object_updated(const autoware_auto_perception_msgs::msg::PredictedObject& prev_object,
                          const autoware_auto_perception_msgs::msg::PredictedObject& current_object)
  {
    // 前回のデータと比較して、位置やサイズが変更されたかどうかを確認する
    const auto& prev_position = prev_object.kinematics.initial_pose_with_covariance.pose.position;
    const auto& current_position = current_object.kinematics.initial_pose_with_covariance.pose.position;

    const auto& prev_velocity = prev_object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto& current_velocity = current_object.kinematics.initial_twist_with_covariance.twist.linear;

    return (prev_position.x != current_position.x ||
            prev_position.y != current_position.y ||
            prev_position.z != current_position.z ||
            prev_velocity.x != current_velocity.x ||
            prev_velocity.y != current_velocity.y ||
            prev_velocity.z != current_velocity.z);
  }

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr data_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr index_publisher_;
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> prev_data_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleUpdateNode>());
  rclcpp::shutdown();
  return 0;
}
