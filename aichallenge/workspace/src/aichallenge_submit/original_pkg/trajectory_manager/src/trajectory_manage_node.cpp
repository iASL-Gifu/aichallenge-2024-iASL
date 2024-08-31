#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "std_msgs/msg/int32.hpp"

class TrajectoryManager : public rclcpp::Node
{
public:
  TrajectoryManager()
  : Node("trajectory_manager")
  {
    // Subscriber to receive Trajectory messages
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", 10, std::bind(&TrajectoryManager::trajectoryCallback, this, std::placeholders::_1));
    
    // Subscriber to receive the event triggering the publish of the Trajectory
    event_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "~/input/object_event", 10, std::bind(&TrajectoryManager::eventCallback, this, std::placeholders::_1));

    // Publisher to publish the stored Trajectory
    trajectory_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", 10);
  }

private:
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    last_trajectory_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(*msg);
    RCLCPP_INFO(this->get_logger(), "Trajectory received and stored.");
  }

  void eventCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    // Check if the event is 3 or 7 and if a trajectory is stored
    if ((msg->data == 3 || msg->data == 7) && last_trajectory_)
    {
      trajectory_publisher_->publish(*last_trajectory_);
      RCLCPP_INFO(this->get_logger(), "Stored Trajectory published.");

      // Reset the stored trajectory after publishing
      last_trajectory_.reset();
      RCLCPP_INFO(this->get_logger(), "Trajectory reset after publishing.");
    }
    else if (!last_trajectory_)
    {
      RCLCPP_WARN(this->get_logger(), "No Trajectory available to publish.");
    }
  }

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr event_subscriber_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> last_trajectory_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryManager>());
  rclcpp::shutdown();
  return 0;
}
