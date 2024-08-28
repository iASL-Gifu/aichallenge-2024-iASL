#include "obstacle_avoidance.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace obstacle_avoidance
{

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance"), costmap_received_(false), path_received_(false)
{
    RCLCPP_INFO(this->get_logger(), "============= Obstacle Avoidance ==============");

    // pub, subの初期化

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "costmap_2d", 1,
        std::bind(&ObstacleAvoidance::costmap_callback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "localization/kinematic_state", 1,
        std::bind(&ObstacleAvoidance::odometry_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<PathWithLaneId>(
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        1, std::bind(&ObstacleAvoidance::path_callback, this, std::placeholders::_1));

    avoidance_path_pub_ = this->create_publisher<PathWithLaneId>("output", 1);

    // パラメータの読み込み
    this->declare_parameter("threshold", 10.0);
    threshold_ = this->get_parameter("threshold").as_double();

    this->declare_parameter("attract", 10.0);
    attract_ = this->get_parameter("attract").as_double();

    this->declare_parameter("repulse", 10.0);
    repulse_ = this->get_parameter("repulse").as_double();

    this->declare_parameter("penalty", 10.0);
    penalty_dist_ = this->get_parameter("penalty").as_double();

    this->declare_parameter("desired_dist", 10.0);
    desired_dist_ = this->get_parameter("desired_dist").as_double();

    this->declare_parameter("start_index", 10);
    start_index_ = this->get_parameter("start_index").as_int();

    this->declare_parameter("num_change_points", 10);
    num_change_points_ = this->get_parameter("num_change_points").as_int();

    this->declare_parameter("lookahead_dist", 10.0);
    lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();

    this->declare_parameter("angle_interval", 10.0);
    angle_interval_ = this->get_parameter("angle_interval").as_double();

    this->declare_parameter("max_angle", 10.0);
    max_angle_ = this->get_parameter("max_angle").as_double();

    this->declare_parameter("min_angle", 10.0);
    min_angle_ = this->get_parameter("min_angle").as_double();

    this->declare_parameter("near_point_dist", 10.0);
    near_point_dist_ = this->get_parameter("near_point_dist").as_double();

    for (double i = min_angle_; i <= max_angle_; i += angle_interval_) {
        angles.push_back(i);
    }
}

void ObstacleAvoidance::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = *msg;
    width_ = costmap_.info.width;
    height_ = costmap_.info.height;
    resolution_ = costmap_.info.resolution;
    origin_x_ = costmap_.info.origin.position.x;
    origin_y_ = costmap_.info.origin.position.y;
    costmap_received_ = true;
}

void ObstacleAvoidance::path_callback(const PathWithLaneId::SharedPtr msg) {
    path_ = *msg;
    path_received_ = true;
}

void ObstacleAvoidance::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry_ = *msg;

    if (!(costmap_received_ && path_received_)) {
        return;
    }

    // threshold以上のコストをもつx,y座標の配列を格納
    // TODO: object以外のox, oyを毎回計算する必要はない
    std::vector<double> ox, oy;
    for (size_t i = 0; i < width_; i++) {
        for (size_t j = 0; j < height_; j++) {
            if (costmap_.data[j * width_ + i] > threshold_) {
                ox.push_back(origin_x_ + i * resolution_);
                oy.push_back(origin_y_ + j * resolution_);
            }
        }
    }
    
    // 回避を開始する点を取得
    auto odom_position = odometry_.pose.pose.position;
    double previous_distance = std::numeric_limits<double>::infinity();
    size_t current_index = 0;
    for (const auto & point : path_.points) {
        auto path_position = point.point.pose.position;
        double distance = std::sqrt(
            std::pow(odom_position.x - path_position.x, 2) +
            std::pow(odom_position.y - path_position.y, 2)
        );

        if (distance > previous_distance) {
            current_index -= 1;
            break;
        }

        previous_distance = distance;
        current_index++;
    }
    size_t start_index = current_index;

    // TODO: 生成した回避経路が毎回リセットされるのはどうにかする
    generate_paths_.clear();
    generate_paths_.push_back(path_.points[current_index]);

    for (int i = 0; i < num_change_points_; i ++) {
        // 現在の位置と姿勢を取得
        double x = generate_paths_[i].point.pose.position.x;
        double y = generate_paths_[i].point.pose.position.y;
        double theta = tf2::getYaw(generate_paths_[i].point.pose.orientation);

        // 車両が目標とする地点を求める
        for (size_t j = current_index; j < path_.points.size(); j++ ) {
            double dist = std::sqrt(
                std::pow(path_.points[j].point.pose.position.x - x, 2) +
                std::pow(path_.points[j].point.pose.position.y - y, 2));

            if (dist > lookahead_dist_) {
                current_index = j;
                break;
            } else {
                continue;
            }
        }
        double goal_x = path_.points[current_index].point.pose.position.x;
        double goal_y = path_.points[current_index].point.pose.position.y;

        // 候補点のpotentialを計算
        std::vector<double> potentials;
        for (const double angle : angles) {
            double angle_rad = (angle * M_PI) / 180.0;
            double new_x = x + desired_dist_ * cos(theta + angle_rad);
            double new_y = x + desired_dist_ * sin(theta + angle_rad);

            double attract = compute_attractive_potential(new_x, new_y, goal_x, goal_y);
            double repulse = compute_repulsive_potential(new_x, new_y, ox, oy);
            double potential = attract_ * attract + repulse_ * repulse;

            potentials.push_back(potential);
        }

        int max_index = std::distance(potentials.begin(), std::max_element(potentials.begin(), potentials.end()));
        double angle_rad = (angles[max_index] * M_PI) / 180.0;

        std::vector<double> orientation;
        euler_to_quaternion(0, 0, angle_rad, orientation);

        PathPointWithLaneId next_point = PathPointWithLaneId();
        next_point.point.pose.position.x = x * desired_dist_ * cos(theta + angle_rad);
        next_point.point.pose.position.y = y * desired_dist_ * sin(theta + angle_rad);
        next_point.point.pose.position.z = 43.1;

        next_point.point.pose.orientation.x = orientation[0];
        next_point.point.pose.orientation.y = orientation[1];
        next_point.point.pose.orientation.z = orientation[2];
        next_point.point.pose.orientation.w = orientation[3];

        next_point.point.longitudinal_velocity_mps = 0.0;
        next_point.point.lateral_velocity_mps = 0.0;
        next_point.point.heading_rate_rps = 0.0;
        next_point.point.is_final = false;

        next_point.lane_ids = path_.points[start_index].lane_ids;

        generate_paths_.push_back(next_point);
    }

    // TODO: 最後のステップに一番近いpointを探す
    int min_goal_index = 0;
    double min_dist = std::numeric_limits<double>::infinity();

    double x = generate_paths_[-1].point.pose.position.x;
    double y = generate_paths_[-1].point.pose.position.y;
    for (size_t i = start_index; i < path_.points.size(); i++) {
        double dist = std::sqrt(
            std::pow(path_.points[i].point.pose.position.x - x, 2) +
            std::pow(path_.points[i].point.pose.position.y - y, 2));

        if (dist <= min_dist) {
            min_dist = dist;
        } else {
            min_goal_index = i + 1;
            break;
        }
    }

    PathWithLaneId new_path = PathWithLaneId();
    new_path.header = path_.header;
    new_path.left_bound = path_.left_bound;
    new_path.right_bound = path_.right_bound;
    // before path
    for (size_t i = 0; i < start_index; i++) {
        new_path.points.push_back(path_.points[i]);
    }

    // generate path
    for (size_t i = 0; i < generate_paths_.size(); i++) {
        new_path.points.push_back(generate_paths_[i]);
    }

    // after path
    for (size_t i = min_goal_index; i < path_.points.size(); i++) {
        new_path.points.push_back(path_.points[i]);
    }

    avoidance_path_pub_->publish(new_path);
}

double ObstacleAvoidance::compute_attractive_potential(double x, double y, double gx, double gy) {
    // ゴールとの距離が小さいほど評価が高い

    return 1.0 / std::sqrt(std::pow(x - gx, 2) + std::pow(y - gy, 2));
}

double ObstacleAvoidance::compute_repulsive_potential(double x, double y, const std::vector<double>& ox, const std::vector<double>& oy) {
    // 障害物との距離が任意距離より近いと減点

    double repulse = 0.0;
    for (size_t i = 0; i < ox.size(); ++i) {
        double dist = std::sqrt(std::pow(x - ox[i], 2) + std::pow(y - oy[i], 2));
        if (dist < penalty_dist_) {
            repulse += -1.0 / dist;
        }
    }

    return repulse;
}

void ObstacleAvoidance::euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result) {
    double phi_half = phi / 2;
    double theta_half = theta / 2;
    double psi_half = psi / 2;

    double cos_phi_half = std::cos(phi_half);
    double sin_phi_half = std::sin(phi_half);
    double cos_theta_half = std::cos(theta_half);
    double sin_theta_half = std::sin(theta_half);
    double cos_psi_half = std::cos(psi_half);
    double sin_psi_half = std::sin(psi_half);

    double w = cos_phi_half * cos_theta_half * cos_psi_half + sin_phi_half * sin_theta_half * sin_psi_half;
    double x = sin_phi_half * cos_theta_half * cos_psi_half - cos_phi_half * sin_theta_half * sin_psi_half;
    double y = cos_phi_half * sin_theta_half * cos_psi_half + sin_phi_half * cos_theta_half * sin_psi_half;
    double z = cos_phi_half * cos_theta_half * sin_psi_half - sin_phi_half * sin_theta_half * cos_psi_half;

    result.push_back(x);
    result.push_back(y);
    result.push_back(z);
    result.push_back(w);

    return;
}

// double ObstacleAvoidance::compute_attractive_potential(
//     geometry_msgs::msg::Pose &start_pose, geometry_msgs::msg::Pose &end_pose)
// {
//     // ゴールとの距離が小さいほど評価が高い
//     start_x = start_pose.position.x;
//     start_y = start_pose.position.y;
//     end_x = end_pose.position.x;
//     end_y = end_pose.position.y;

//     return 1.0 - / std::sqrt(std::pow(start_x - end_x, 2) + std::pow(start_y, end_y, 2));
// }

// double ObstacleAvoidance::compute_repulsive_potential(
//     geometry_msgs::msg::Pose &start_pose,
//     const std::vector<double> &ox, const std::vector<double> &oy)
// {
//     // 障害物との距離が任意距離より近いと減点
//     double repulse = 0.0;
//     for (size_t i = 0; i < ox.size(); i++) {
//         double dist = std::sqrt(
//             std::pow(start_pose.position.x - ox[i], 2) +
//             std::pow(start_pose.position.y - oy[i], 2));

//         if (dist < penalty_dist_) {
//             repulse += -1.0 / dist;
//         }
//     }

//     return repulse;
// }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_avoidance::ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}