#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "nav_msgs/msg/odometry.hpp"  // Odometry message
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"  // To calculate distance between poses
#include "visualization_msgs/msg/marker.hpp"  // To visualize the goal pose

class TrajectoryManager : public rclcpp::Node
{
public:
  TrajectoryManager()
  : Node("trajectory_manager"), planning_active_(true), distance_threshold_(5.0)
  {
    this->declare_parameter("distance_threshold", 0.0);
    this->get_parameter("distance_threshold", distance_threshold_);

    // Subscriber to receive Trajectory messages
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", 10, std::bind(&TrajectoryManager::trajectoryCallback, this, std::placeholders::_1));
    
    // Subscriber to receive the LaneletRoute message
    route_subscriber_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "~/input/route", 10, std::bind(&TrajectoryManager::routeCallback, this, std::placeholders::_1));

    // Subscriber to receive Odometry (kinematics) messages
    sub_kinematics_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematics", 1, std::bind(&TrajectoryManager::kinematicsCallback, this, std::placeholders::_1));

    // Publisher to publish the stored Trajectory
    trajectory_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", 10);

    // Publisher for visualizing the goal pose
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/output/goal_marker", 10);
  }

private:
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    if (planning_active_) 
    {
      last_trajectory_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(*msg);
      planning_active_ = false;  // Disable further planning until the trajectory is published
    }
  }

  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg)
  {
    // Store the new goal pose in pending_goal_pose_
    pending_goal_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->start_pose);

    // If current_goal_pose_ is not set, initialize it
    if (!current_goal_pose_)
    {
      current_goal_pose_ = pending_goal_pose_;
      pending_goal_pose_.reset();
    }

    // Visualize the goal pose
    publishGoalMarker(*current_goal_pose_);
  }

  void kinematicsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store or process the received odometry data
    odometry_ = msg;

    // If current goal pose is available, calculate distance
    if (current_goal_pose_ && last_trajectory_)
    {
      double distance_to_goal = calculateDistance(odometry_->pose.pose, *current_goal_pose_);

      // Check if distance is within the threshold
      if (distance_to_goal <= distance_threshold_)
      {
        // Publish the stored trajectory if within threshold
        trajectory_publisher_->publish(*last_trajectory_);
        last_trajectory_.reset();  // Reset after publishing
        planning_active_ = true;   // Reactivate planning
        RCLCPP_INFO(this->get_logger(), "Trajectory published.");

        // If a new pending goal is available, update current goal
        if (pending_goal_pose_)
        {
          current_goal_pose_ = pending_goal_pose_;
          pending_goal_pose_.reset();  // Clear pending goal after applying

          // Visualize the new goal pose
          publishGoalMarker(*current_goal_pose_);
        }else{
          current_goal_pose_.reset();
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Trajectory not published, distance to goal: %f", distance_to_goal);
      }
    }
  }

  // Utility function to calculate distance between two poses
  double calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
  {
    return std::sqrt(std::pow(pose1.position.x - pose2.position.x, 2) +
                     std::pow(pose1.position.y - pose2.position.y, 2));
  }

  // Function to publish the goal pose as a marker
  void publishGoalMarker(const geometry_msgs::msg::Pose &goal_pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // Change to the appropriate frame
    marker.header.stamp = this->now();
    marker.ns = "goal_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = goal_pose;

    // Set the scale of the marker (size of the sphere)
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color (yellow, fully opaque)
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Set the lifetime of the marker (keep it visible indefinitely)
    marker.lifetime = rclcpp::Duration(0, 0);

    // Publish the marker
    marker_publisher_->publish(marker);
  }

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;  // Publisher for goal markers

  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> last_trajectory_;
  std::shared_ptr<geometry_msgs::msg::Pose> current_goal_pose_;  // Current goal pose
  std::shared_ptr<geometry_msgs::msg::Pose> pending_goal_pose_;  // Pending goal pose
  nav_msgs::msg::Odometry::SharedPtr odometry_;  // Variable to store the odometry data
  bool planning_active_;
  double distance_threshold_;  // Threshold distance to goal
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryManager>());
  rclcpp::shutdown();
  return 0;
}
