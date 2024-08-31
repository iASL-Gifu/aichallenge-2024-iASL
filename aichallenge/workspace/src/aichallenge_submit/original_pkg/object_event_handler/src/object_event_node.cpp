#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

class ObstacleUpdateNode : public rclcpp::Node
{
public:
  ObstacleUpdateNode() : Node("obstacle_update_node")
  {
    // サブスクライバを作成
    data_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/input/objects", 10,
        std::bind(&ObstacleUpdateNode::data_callback, this, std::placeholders::_1));

    // パブリッシャを作成
    index_publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output/object_event", 10);

    // 前の時刻のデータを初期化
    prev_data_.resize(32, 0.0); // 8つの障害物 * 4つの属性 (x, y, z, radius)
  }

private:
  void data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // 現在のデータを取得
    auto current_data = msg->data;

    // 障害物データの更新を確認
    for (size_t i = 0; i < current_data.size(); i += 4)
    {
      if (prev_data_[i] != current_data[i] || prev_data_[i + 1] != current_data[i + 1] ||
          prev_data_[i + 2] != current_data[i + 2] || prev_data_[i + 3] != current_data[i + 3])
      {
        RCLCPP_INFO(this->get_logger(), "Obstacle %zu updated", i / 4);

        // 更新された障害物のインデックスをPub
        std_msgs::msg::Int32 index_msg;
        index_msg.data = i / 4;
        index_publisher_->publish(index_msg);
      }
    }

    // 現在のデータを保存して次のループに備える
    prev_data_ = current_data;
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr index_publisher_;
  std::vector<double> prev_data_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleUpdateNode>());
  rclcpp::shutdown();
  return 0;
}
