#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "std_msgs/msg/int32.hpp"

class ControlModeMonitor : public rclcpp::Node
{
public:
    ControlModeMonitor()
    : Node("control_mode_monitor")
    {
        // サブスクライバーの設定
        control_mode_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
            "~/input/control_mode", 10,
            std::bind(&ControlModeMonitor::controlModeCallback, this, std::placeholders::_1));

        // パブリッシャーの設定
        mode_change_pub_ = this->create_publisher<std_msgs::msg::Int32>("~/output/mode_change_event", 10);

        // 初期化
        last_control_mode_ = -1; // 初期値（不正な値として-1を使用）
    }

private:
    void controlModeCallback(const autoware_auto_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
    {
        if (last_control_mode_ != msg->mode) {
            last_control_mode_ = msg->mode;
            std_msgs::msg::Int32 event_msg;
            event_msg.data = 1;  // モードが変更されたことを示すイベント
            mode_change_pub_->publish(event_msg);
        }
    }

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_change_pub_;
    int last_control_mode_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlModeMonitor>());
    rclcpp::shutdown();
    return 0;
}
