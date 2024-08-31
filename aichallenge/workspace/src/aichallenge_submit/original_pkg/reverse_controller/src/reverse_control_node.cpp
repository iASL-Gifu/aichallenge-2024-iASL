#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <std_msgs/msg/int32.hpp>

class ControlCommandReverser : public rclcpp::Node
{
public:
  ControlCommandReverser() : Node("control_command_reverser")
  {
    // パラメータの宣言と初期化
    this->declare_parameter<double>("speed_threshold", 0.5);
    
    // Control Command Subscriber
    sub_cmd_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "input/control_cmd", 1,
      std::bind(&ControlCommandReverser::on_control_cmd_received, this, std::placeholders::_1));
    
    // Mode Change Event Subscriber
    sub_event_ = this->create_subscription<std_msgs::msg::Int32>(
      "~/input/mode_change_event", 1,
      std::bind(&ControlCommandReverser::on_mode_change_event_received, this, std::placeholders::_1));

    // Control Mode Subscriber
    sub_mode_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "~/input/control_mode", 1,
      std::bind(&ControlCommandReverser::on_mode_received, this, std::placeholders::_1));

    // Control Command Publisher
    pub_cmd_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("output/control_cmd", 1);
  }

private:
  void on_control_cmd_received(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received control command: acceleration = %f, steering_angle = %f", 
                msg->longitudinal.acceleration, msg->lateral.steering_tire_angle);
    RCLCPP_INFO(this->get_logger(), "Mode changed flag: %d, Current mode: %d", mode_changed_, current_mode_);

    if (mode_changed_ == 1 && current_mode_ == 1) {
      RCLCPP_INFO(this->get_logger(), "Adjusting steering angle to -0.5 radians");
      // 左に0.5ラジアン切る
      msg->lateral.steering_tire_angle = -0.5;  
      mode_changed_ = 0;  // ステアリングを送信したことを記録
    }

    // 加速度を反転
    msg->longitudinal.acceleration = -(msg->longitudinal.acceleration);
    RCLCPP_INFO(this->get_logger(), "Reversed acceleration: %f", msg->longitudinal.acceleration);

    // 修正されたコマンドをパブリッシュ
    pub_cmd_->publish(*msg);
  }

  void on_mode_received(const autoware_auto_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
  {
    current_mode_ = msg->mode;
    RCLCPP_INFO(this->get_logger(), "Received control mode: %d", current_mode_);
  }

  void on_mode_change_event_received(const std_msgs::msg::Int32::SharedPtr msg)
  {
    mode_changed_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received mode change event: %d", mode_changed_);
  }

  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_event_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr sub_mode_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_cmd_;
  
  int mode_changed_ = 0;
  int current_mode_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlCommandReverser>());
  rclcpp::shutdown();
  return 0;
}
