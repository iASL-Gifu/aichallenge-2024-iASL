#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

class TrajectorySelector : public rclcpp::Node
{
public:
  TrajectorySelector() : Node("trajectory_selector")
  {
    scenario_subscriber_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
      "/scenario", 10, std::bind(&TrajectorySelector::scenarioCallback, this, std::placeholders::_1));

    status_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/status", 10, std::bind(&TrajectorySelector::statusCallback, this, std::placeholders::_1));

    pit_trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/trajectory_pit", 10, std::bind(&TrajectorySelector::pitTrajectoryCallback, this, std::placeholders::_1));

    lane_trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/trajectory_lane", 10, std::bind(&TrajectorySelector::laneTrajectoryCallback, this, std::placeholders::_1));

    avoidance_trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/trajectory_avoidance", 10, std::bind(&TrajectorySelector::avoidanceTrajectoryCallback, this, std::placeholders::_1));

    selected_trajectory_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "/selected_trajectory", 10);
  }

private:
  void statusCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() > 3)
    {
      current_section_ = msg->data[3];
    }
  }

  void scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg)
  {
    current_scenario_ = msg->current_scenario;
    publishSelectedTrajectory();
  }

  void pitTrajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    pit_trajectory_ = msg;
    publishSelectedTrajectory();
  }

  void laneTrajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    lane_trajectory_ = msg;
    publishSelectedTrajectory();
  }

  void avoidanceTrajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    avoidance_trajectory_ = msg;
    publishSelectedTrajectory();
  }

  void publishSelectedTrajectory()
  {
    if (current_scenario_.empty()) {
      return;
    }

    autoware_auto_planning_msgs::msg::Trajectory::SharedPtr selected_trajectory;

    if (current_scenario_ == "Parking" && current_section_ == 8 && pit_trajectory_) {
      selected_trajectory = pit_trajectory_;
    } else if (current_scenario_ == "LaneDriving" && lane_trajectory_) {
      selected_trajectory = lane_trajectory_;
    } else if (current_scenario_ == "Avoidance" && avoidance_trajectory_) {
      selected_trajectory = avoidance_trajectory_;
    } else if (!pit_trajectory_ && lane_trajectory_) {
      selected_trajectory = lane_trajectory_;
    } else if (pit_trajectory_ && !lane_trajectory_) {
      selected_trajectory = pit_trajectory_;
    } else {
      return;
    }

    selected_trajectory_publisher_->publish(*selected_trajectory);
  }

  rclcpp::Subscription<tier4_planning_msgs::msg::Scenario>::SharedPtr scenario_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pit_trajectory_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr lane_trajectory_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr avoidance_trajectory_subscriber_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr selected_trajectory_publisher_;

  std::string current_scenario_;
  float current_section_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr pit_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr lane_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr avoidance_trajectory_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySelector>());
  rclcpp::shutdown();
  return 0;
}
