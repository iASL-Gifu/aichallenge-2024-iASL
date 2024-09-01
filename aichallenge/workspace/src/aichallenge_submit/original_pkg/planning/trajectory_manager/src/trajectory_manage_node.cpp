#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"

class TrajectoryManager : public rclcpp::Node
{
public:
  TrajectoryManager()
  : Node("trajectory_manager"), planning_active_(true)
  {
    // Subscriber to receive Trajectory messages
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", 10, std::bind(&TrajectoryManager::trajectoryCallback, this, std::placeholders::_1));
    
    // Subscriber to receive the LaneletRoute message
    route_subscriber_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "~/input/route", 10, std::bind(&TrajectoryManager::routeCallback, this, std::placeholders::_1));

    // Publisher to publish the stored Trajectory
    trajectory_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", 10);
  }

private:
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    if (planning_active_) 
    {
      last_trajectory_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(*msg);
      RCLCPP_INFO(this->get_logger(), "Trajectory received and stored.");
      // Disable further planning until the trajectory is published
      planning_active_ = false;
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Planning is inactive. Ignoring received trajectory.");
    }
  }

  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg)
  {
    // Check if a trajectory is stored
    if (last_trajectory_)
    {
      trajectory_publisher_->publish(*last_trajectory_);
      RCLCPP_INFO(this->get_logger(), "Stored Trajectory published.");

      // Reset the stored trajectory after publishing and reactivate planning
      last_trajectory_.reset();
      RCLCPP_INFO(this->get_logger(), "Trajectory reset after publishing.");

      // Reactivate planning
      planning_active_ = true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No Trajectory available to publish.");
    }
  }

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> last_trajectory_;
  bool planning_active_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryManager>());
  rclcpp::shutdown();
  return 0;
}
