#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

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
    
    // Velocity Report Subscriber
    sub_velocity_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      "input/velocity_report", 1,
      std::bind(&ControlCommandReverser::on_velocity_report_received, this, std::placeholders::_1));
    
    // Control Command Publisher
    pub_cmd_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("output/control_cmd", 1);

    // パラメータの取得
    speed_threshold_ = this->get_parameter("speed_threshold").as_double();
  }

private:
  void on_control_cmd_received(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    last_cmd_ = *msg;
    // 速度がしきい値以下かつ、まだステアリングを送信していない場合
    if (current_velocity_ < speed_threshold_ && !steering_sent_) {
      // 左に0.5ラジアン切る
      last_cmd_.lateral.steering_tire_angle = -0.5;  
      steering_sent_ = true;  // ステアリングを送信したことを記録
    }

    last_cmd_.longitudinal.acceleration = -(msg->longitudinal.acceleration);

    // コマンドをパブリッシュ
    pub_cmd_->publish(last_cmd_);
  }

  void on_velocity_report_received(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
  {
    current_velocity_ = msg->longitudinal_velocity;
    
    // 速度がしきい値を超えた場合、ステアリングをリセットする準備をする
    if (current_velocity_ >= speed_threshold_) {
      steering_sent_ = false;
    }
  }

  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_cmd_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_cmd_;
  
  autoware_auto_control_msgs::msg::AckermannControlCommand last_cmd_;
  double current_velocity_{0.0};
  double speed_threshold_;
  bool steering_sent_{false};  // ステアリングが一度送信されたかどうかを追跡
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlCommandReverser>());
  rclcpp::shutdown();
  return 0;
}
