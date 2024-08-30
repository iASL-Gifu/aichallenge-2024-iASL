#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

class TrajectorySelector : public rclcpp::Node
{
public:
  TrajectorySelector()
  : Node("trajectory_selector"),
    previous_section_(-1) // 初期値として無効なセクションを設定
  {
    status_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "~/input/status", 10, std::bind(&TrajectorySelector::status_callback, this, std::placeholders::_1));

    section_change_publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output/section_change", 10);
  }

private:
  void status_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() > 3)
    {
      int current_section = static_cast<int>(msg->data[3]);

      // セクションが4から5に変わった場合
      if (previous_section_ == 4 && current_section == 5)
      {
        publish_event(45); // 4->5 の場合は 45
      }
      // セクションが8から0に変わった場合
      else if (previous_section_ == 8 && current_section == 1)
      {
        publish_event(80); // 8->0 の場合は 80
      }

      // 現在のセクションを記録
      previous_section_ = current_section;
    }
  }

  void publish_event(int event_value)
  {
    auto message = std_msgs::msg::Int32();
    message.data = event_value;
    section_change_publisher_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr section_change_publisher_;
  int previous_section_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySelector>());
  rclcpp::shutdown();
  return 0;
}