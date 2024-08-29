#include "obstacle_avoidance.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <utility>

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

    // TODO: laneletを使用したpath
    // path_sub_ = this->create_subscription<PathWithLaneId>(
    //     "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
    //     1, std::bind(&ObstacleAvoidance::path_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "vechile_path", 1, std::bind(&ObstacleAvoidance::path_callback, this, std::placeholders::_1));

    // TODO: laneletを使用したpath
    // avoidance_path_pub_ = this->create_publisher<PathWithLaneId>("output", 1);

    avoidance_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("obstacle_avoidance", 1);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    costmap_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "costmap_2d_2", qos
    );

    // パラメータの読み込み

    // TODO: 消す
    this->declare_parameter("threshold", 10.0);
    threshold_ = this->get_parameter("threshold").as_double();

    this->declare_parameter("attract", 10.0);
    attract_ = this->get_parameter("attract").as_double();

    this->declare_parameter("repulse", 10.0);
    repulse_ = this->get_parameter("repulse").as_double();

    // TODO:
    this->declare_parameter("penalty_dist", 10.0);
    penalty_dist_ = this->get_parameter("penalty_dist").as_double();

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

    // TODO
    this->declare_parameter("near_point_dist", 10.0);
    near_point_dist_ = this->get_parameter("near_point_dist").as_double();

    this->declare_parameter("margin", 10);
    margin_ = this->get_parameter("margin").as_int();

    this->declare_parameter("forward_dist", 10.0);
    forward_dist_ = this->get_parameter("forward_dist").as_double();

    this->declare_parameter("side_dist", 10.0);
    side_dist_ = this->get_parameter("side_dist").as_double();

    this->declare_parameter("visual", true);
    visual_ = this->get_parameter("visual").as_bool();

    this->declare_parameter("visual_angle", 10.0);
    visual_angle_ = this->get_parameter("visual_angle").as_double();

    this->declare_parameter("min", 10.0);
    min_ = this->get_parameter("min").as_double();

    this->declare_parameter("max", 10.0);
    max_ = this->get_parameter("max").as_double();

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

    int index = 90 * width_ + 230;
    RCLCPP_INFO(this->get_logger(), "xxxxxxxxxxxxxxxx %d", costmap_.data[index]);
}

// void ObstacleAvoidance::path_callback(const PathWithLaneId::SharedPtr msg) {
//     path_ = *msg;
//     path_received_ = true;
// }

void ObstacleAvoidance::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    path_received_ = true;
}

void ObstacleAvoidance::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry_ = *msg;

    if (!(costmap_received_ && path_received_)) {
        return;
    }
    
    // 回避を開始する点を取得
    auto odom_position = odometry_.pose.pose.position;
    double previous_distance = std::numeric_limits<double>::infinity();
    size_t current_index = 0;
    for (const auto & point : path_.poses) {
        auto path_position = point.pose.position;

        double distance = std::sqrt(
            std::pow(odom_position.x - path_position.x, 2) +
            std::pow(odom_position.y - path_position.y, 2)
        );

        if (current_index < 10) {
            RCLCPP_INFO(this->get_logger(), "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
            RCLCPP_INFO(this->get_logger(), "Point x: %f, y: %f, distance: %f", path_position.x, path_position.y, distance);
        }

        if (distance > previous_distance) {
            current_index -= 1;
            break;
        }

        previous_distance = distance;
        current_index++;
    }
    size_t start_index = current_index;

    generate_paths_.clear();
    generate_paths_.push_back(path_.poses[current_index]);

    for (int i = 0; i < num_change_points_; i ++) {
        // 現在の位置と姿勢を取得
        double x = generate_paths_[i].pose.position.x;
        double y = generate_paths_[i].pose.position.y;
        double theta = tf2::getYaw(generate_paths_[i].pose.orientation);

        // 車両が目標とする地点を求める
        for (size_t j = current_index; j < path_.poses.size(); j++ ) {
            double dist = std::sqrt(
                std::pow(path_.poses[j].pose.position.x - x, 2) +
                std::pow(path_.poses[j].pose.position.y - y, 2));

            if (dist > lookahead_dist_) {
                current_index = j;
                break;
            } else {
                continue;
            }
        }
        double goal_x = path_.poses[current_index].pose.position.x;
        double goal_y = path_.poses[current_index].pose.position.y;

        // 候補点のpotentialを計算
        std::vector<double> potentials;
        for (const double angle : angles) {
            double angle_rad = (angle * M_PI) / 180.0;
            double new_x = x + desired_dist_ * cos(theta + angle_rad);
            double new_y = y + desired_dist_ * sin(theta + angle_rad);

            double attract = compute_attractive_potential(new_x, new_y, goal_x, goal_y);
            double repulse = compute_repulsive_potential(new_x, new_y, theta + angle_rad);
            double potential = attract_ * attract + repulse_ * repulse;

            potentials.push_back(potential);
        }

        int max_index = std::distance(potentials.begin(), std::max_element(potentials.begin(), potentials.end()));
        double angle_rad = (angles[max_index] * M_PI) / 180.0;

        std::vector<double> orientation;
        euler_to_quaternion(0, 0, angle_rad + theta, orientation);

        // TODO: 消す
        // double sample_x = x + desired_dist_ * cos(theta + angle_rad);
        // double sample_y = y + desired_dist_ * sin(theta + angle_rad);
        // double sample_angel = angles[max_index];

        // if (counter_ < aa) {
        //     RCLCPP_INFO(this->get_logger(), "next_x: %f, next_y: %f, next_angle: %f", sample_x, sample_y, sample_angel);
        // }

        geometry_msgs::msg::PoseStamped next_point = geometry_msgs::msg::PoseStamped();
        next_point.pose.position.x = x + desired_dist_ * cos(theta + angle_rad);
        next_point.pose.position.y = y + desired_dist_ * sin(theta + angle_rad);
        next_point.pose.position.z = 43.1;

        next_point.pose.orientation.x = orientation[0];
        next_point.pose.orientation.y = orientation[1];
        next_point.pose.orientation.z = orientation[2];
        next_point.pose.orientation.w = orientation[3];

        generate_paths_.push_back(next_point);
    }

    // 最後のステップに一番近いpointを探す
    int min_goal_index = 100;
    double min_dist = std::numeric_limits<double>::infinity();

    double x = generate_paths_[num_change_points_].pose.position.x;
    double y = generate_paths_[num_change_points_].pose.position.y;
    for (size_t i = start_index + margin_; i < path_.poses.size(); i++) {
        double dist = std::sqrt(
            std::pow(path_.poses[i].pose.position.x - x, 2) +
            std::pow(path_.poses[i].pose.position.y - y, 2));

        if (dist <= min_dist) {
            min_dist = dist;
        } else {
            min_goal_index = i + 1;
            break;
        }
    }

    // RCLCPP_INFO(this->get_logger(), "=======%d=======", min_goal_index);

    nav_msgs::msg::Path new_path = nav_msgs::msg::Path();
    new_path.header = path_.header;

    // before path
    for (size_t i = 0; i < start_index; i++) {
        new_path.poses.push_back(path_.poses[i]);
    }
    // generate path
    for (size_t i = 0; i < generate_paths_.size(); i++) {
        new_path.poses.push_back(generate_paths_[i]);
    }
    // after path
    for (size_t i = min_goal_index; i < path_.poses.size(); i++) {
        new_path.poses.push_back(path_.poses[i]);
    }

    avoidance_path_pub_->publish(new_path);

    counter_++;
}

// void ObstacleAvoidance::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     odometry_ = *msg;

//     if (!(costmap_received_ && path_received_)) {
//         return;
//     }
    
//     // 回避を開始する点を取得
//     auto odom_position = odometry_.pose.pose.position;
//     double previous_distance = std::numeric_limits<double>::infinity();
//     size_t current_index = 0;
//     for (const auto & point : path_.points) {
//         auto path_position = point.point.pose.position;
//         double distance = std::sqrt(
//             std::pow(odom_position.x - path_position.x, 2) +
//             std::pow(odom_position.y - path_position.y, 2)
//         );

//         if (distance > previous_distance) {
//             current_index -= 1;
//             break;
//         }

//         previous_distance = distance;
//         current_index++;
//     }
//     size_t start_index = current_index;

//     // TODO: 生成した回避経路が毎回リセットされるのはどうにかする
//     generate_paths_.clear();
//     generate_paths_.push_back(path_.points[current_index]);

//     for (int i = 0; i < num_change_points_; i ++) {
//         // 現在の位置と姿勢を取得
//         double x = generate_paths_[i].point.pose.position.x;
//         double y = generate_paths_[i].point.pose.position.y;
//         double theta = tf2::getYaw(generate_paths_[i].point.pose.orientation);

//         // 車両が目標とする地点を求める
//         for (size_t j = current_index; j < path_.points.size(); j++ ) {
//             double dist = std::sqrt(
//                 std::pow(path_.points[j].point.pose.position.x - x, 2) +
//                 std::pow(path_.points[j].point.pose.position.y - y, 2));

//             if (dist > lookahead_dist_) {
//                 current_index = j;
//                 break;
//             } else {
//                 continue;
//             }
//         }
//         double goal_x = path_.points[current_index].point.pose.position.x;
//         double goal_y = path_.points[current_index].point.pose.position.y;

//         // 候補点のpotentialを計算
//         std::vector<double> potentials;
//         for (const double angle : angles) {
//             double angle_rad = (angle * M_PI) / 180.0;
//             double new_x = x + desired_dist_ * cos(theta + angle_rad);
//             double new_y = y + desired_dist_ * sin(theta + angle_rad);

//             double attract = compute_attractive_potential(new_x, new_y, goal_x, goal_y);
//             double repulse = compute_repulsive_potential(new_x, new_y, theta + angle_rad);
//             double potential = attract_ * attract + repulse_ * repulse;

//             potentials.push_back(potential);
//         }

//         int max_index = std::distance(potentials.begin(), std::max_element(potentials.begin(), potentials.end()));
//         double angle_rad = (angles[max_index] * M_PI) / 180.0;

//         std::vector<double> orientation;
//         euler_to_quaternion(0, 0, angle_rad + theta, orientation);

//         // TODO: 消す
//         // double sample_x = x + desired_dist_ * cos(theta + angle_rad);
//         // double sample_y = y + desired_dist_ * sin(theta + angle_rad);
//         // double sample_angel = angles[max_index];

//         // if (counter_ < aa) {
//         //     RCLCPP_INFO(this->get_logger(), "next_x: %f, next_y: %f, next_angle: %f", sample_x, sample_y, sample_angel);
//         // }

//         PathPointWithLaneId next_point = PathPointWithLaneId();
//         next_point.point.pose.position.x = x + desired_dist_ * cos(theta + angle_rad);
//         next_point.point.pose.position.y = y + desired_dist_ * sin(theta + angle_rad);
//         next_point.point.pose.position.z = 43.1;

//         next_point.point.pose.orientation.x = orientation[0];
//         next_point.point.pose.orientation.y = orientation[1];
//         next_point.point.pose.orientation.z = orientation[2];
//         next_point.point.pose.orientation.w = orientation[3];

//         next_point.point.longitudinal_velocity_mps = 0.0;
//         next_point.point.lateral_velocity_mps = 0.0;
//         next_point.point.heading_rate_rps = 0.0;
//         next_point.point.is_final = false;

//         next_point.lane_ids = path_.points[start_index].lane_ids;

//         generate_paths_.push_back(next_point);
//     }

//     // 最後のステップに一番近いpointを探す
//     int min_goal_index = 100;
//     double min_dist = std::numeric_limits<double>::infinity();

//     double x = generate_paths_[num_change_points_].point.pose.position.x;
//     double y = generate_paths_[num_change_points_].point.pose.position.y;
//     for (size_t i = start_index + margin_; i < path_.points.size(); i++) {
//         double dist = std::sqrt(
//             std::pow(path_.points[i].point.pose.position.x - x, 2) +
//             std::pow(path_.points[i].point.pose.position.y - y, 2));

//         if (dist <= min_dist) {
//             min_dist = dist;
//         } else {
//             min_goal_index = i + 1;
//             break;
//         }
//     }

//     // RCLCPP_INFO(this->get_logger(), "=======%d=======", min_goal_index);

//     PathWithLaneId new_path = PathWithLaneId();
//     new_path.header = path_.header;
//     new_path.left_bound = path_.left_bound;
//     new_path.right_bound = path_.right_bound;

//     // before path
//     for (size_t i = 0; i < start_index; i++) {
//         new_path.points.push_back(path_.points[i]);
//     }
//     // generate path
//     for (size_t i = 0; i < generate_paths_.size(); i++) {
//         new_path.points.push_back(generate_paths_[i]);
//     }
//     // after path
//     for (size_t i = min_goal_index; i < path_.points.size(); i++) {
//         new_path.points.push_back(path_.points[i]);
//     }

//     avoidance_path_pub_->publish(new_path);

//     counter_++;
// }

double ObstacleAvoidance::compute_attractive_potential(double x, double y, double gx, double gy) {
    // ゴールとの距離が小さいほど評価が高い

    return 1.0 / std::sqrt(std::pow(x - gx, 2) + std::pow(y - gy, 2));
}

// TODO: 消す
// double ObstacleAvoidance::compute_repulsive_potential(double x, double y, const std::vector<double>& ox, const std::vector<double>& oy) {
//     // 障害物との距離が任意距離より近いと減点

//     double repulse = 0.0;
//     for (size_t i = 0; i < ox.size(); ++i) {
//         double dist = std::sqrt(std::pow(x - ox[i], 2) + std::pow(y - oy[i], 2));
//         if (dist < penalty_dist_) {
//             repulse += -1.0 / dist;
//         }
//     }

//     return repulse;
// }

double ObstacleAvoidance::compute_repulsive_potential(double x, double y, double theta)
{
    // インデックスを計算
    int x_start = (x - side_dist_ - origin_x_) / resolution_;
    int x_end = (x + side_dist_ - origin_x_) / resolution_;
    int y_start = (y - origin_y_) / resolution_;
    int y_end = (y + forward_dist_ - origin_y_) / resolution_;

    int center_x = (x - origin_x_) /resolution_;
    int center_y = y_start;

    double repulse = 0.0;
    nav_msgs::msg::OccupancyGrid costmap = costmap_;

    for (int i = x_start; i < x_end; i++) {
        for (int j = y_start; j < y_end; j++) {
            std::pair<int, int> rotated_point = rotatePoint(i, j, theta - 1.75, center_x, center_y);
            int new_x = rotated_point.first;
            int new_y = rotated_point.second;
            int index = new_y * width_ + new_x;

            if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_) {
                // double aaa = costmap_.data[index];
                // if (aaa != 127) {
                //     repulse -= costmap_.data[index];
                // }
                // double cost = costmap_.data[index];
                // if (cost > min_ && cost < max_) {
                //     repulse++;
                // }
                repulse -= costmap_.data[index];
            }

            if (visual_ && (i == x_start || i == x_end - 1 || j == y_start || j == y_end - 1)) {
                costmap.data[index] = near_point_dist_;
            }
        }
    }

    costmap_2d_pub_->publish(costmap);

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

std::pair<int, int> ObstacleAvoidance::rotatePoint(double x, double y, double theta, double center_x, double center_y)
{
    // 座標を中心からの相対位置に変換
    double rel_x = x - center_x;
    double rel_y = y - center_y;

    // 相対位置で回転を適用
    double x_new = rel_x * std::cos(theta) - rel_y * std::sin(theta);
    double y_new = rel_x * std::sin(theta) + rel_y * std::cos(theta);

    // 元の位置に戻す
    x_new += center_x;
    y_new += center_y;

    return std::make_pair(std::floor(x_new), std::floor(y_new));
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_avoidance::ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}