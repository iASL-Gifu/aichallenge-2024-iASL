#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <rclcpp/rclcpp.hpp>

class ParkingController : public rclcpp::Node {
public:
    ParkingController()
    : Node("parking_controller"), pit_status_(0.0)  // pit_status_を初期化
    {
        // パラメータの宣言
        this->declare_parameter<double>("target_x", 10.0);
        this->declare_parameter<double>("target_y", 5.0);
        this->declare_parameter<double>("threshold_distance", 1.0);

        // パラメータの取得
        this->get_parameter("target_x", target_position_.x);
        this->get_parameter("target_y", target_position_.y);
        this->get_parameter("threshold_distance", threshold_distance_);

        // シナリオのサブスクライブ
        scenario_sub_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
            "/scenario", 10, std::bind(&ParkingController::scenarioCallback, this, std::placeholders::_1));

        // Odomのサブスクライブ
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "~/input/kinematics", 10, std::bind(&ParkingController::odomCallback, this, std::placeholders::_1));

        // pitStatusのサブスクライブ
        pit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "~/input/pit_status", 10, std::bind(&ParkingController::pitCallback, this, std::placeholders::_1));

        // ギアコマンドのパブリッシャー
        gear_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>(
            "/gear_command", 10);
    }

private:
    void scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg) {
        current_scenario_ = msg;
    }

    void pitCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        pit_status_ = msg->data;
        if (pit_status_ >= 3.0) {
            RCLCPP_INFO(this->get_logger(), "Pit status is 3 or more, setting gear to Drive.");
            setGearToDrive();
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odometry_ = msg;

        if (current_scenario_ && current_scenario_->current_scenario == "Parking") {
            double distance = calculateDistance(odometry_->pose.pose.position, target_position_);
            if (distance <= threshold_distance_) {
                // ここでギアをParkingに設定する処理を追加
                RCLCPP_INFO(this->get_logger(), "In Parking scenario and within threshold distance, setting gear to Parking.");
                setGearToParking();
            }
        }
    }

    double calculateDistance(const geometry_msgs::msg::Point& pos1, const geometry_msgs::msg::Point& pos2) {
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) + std::pow(pos1.y - pos2.y, 2));
    }

    void setGearToParking() {
        auto gear_msg = autoware_auto_vehicle_msgs::msg::GearCommand();
        gear_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;  // Parkingギアに設定
        gear_pub_->publish(gear_msg);
    }

    void setGearToDrive() {
        auto gear_msg = autoware_auto_vehicle_msgs::msg::GearCommand();
        gear_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;  // Driveギアに設定
        gear_pub_->publish(gear_msg);
    }

    rclcpp::Subscription<tier4_planning_msgs::msg::Scenario>::SharedPtr scenario_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pit_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_pub_;
    
    tier4_planning_msgs::msg::Scenario::SharedPtr current_scenario_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;
    float pit_status_;  // pit_status_の変数を追加

    geometry_msgs::msg::Point target_position_;
    double threshold_distance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParkingController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
