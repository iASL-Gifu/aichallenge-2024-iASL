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

    ex_x_ = 0.0;
    ex_y_ = 0.0;
}

bool ObstacleAvoidance::are_positions_equal() {
    // RCLCPP_INFO(this->get_logger(), 
    //         "Current Odometry Position: x: %f, y: %f, Previous Odometry Position: x: %f, y: %f", 
    //         odometry_.pose.pose.position.x, 
    //         odometry_.pose.pose.position.y, 
    //         ex_x_, 
    //         ex_y_);

    double dx = odometry_.pose.pose.position.x - ex_x_;
    double dy = odometry_.pose.pose.position.y - ex_y_;

    return (std::sqrt(dx * dx + dy * dy) <= 0.01);
}

void ObstacleAvoidance::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = *msg;
    width_ = costmap_.info.width;
    height_ = costmap_.info.height;
    resolution_ = costmap_.info.resolution;
    origin_x_ = costmap_.info.origin.position.x;
    origin_y_ = costmap_.info.origin.position.y;
    costmap_received_ = true;

    // int index = 90 * width_ + 230;
    // RCLCPP_INFO(this->get_logger(), "xxxxxxxxxxxxxxxx %d", costmap_.data[index]);
}

// void ObstacleAvoidance::path_callback(const PathWithLaneId::SharedPtr msg) {
//     path_ = *msg;
//     path_received_ = true;
// }

void ObstacleAvoidance::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    path_received_ = true;

    // double x = msg->poses[0].pose.position.x;
    // double y = msg->poses[0].pose.position.y;

    // RCLCPP_INFO(this->get_logger(), "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
    // RCLCPP_INFO(this->get_logger(), "Index 0 Position: x: %f, y: %f", x, y);
}

void ObstacleAvoidance::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Odometry Callback");
    odometry_ = *msg;

    if (!costmap_received_ || !path_received_) {
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "zzzzzzzzzzzzzzzzzzzzzzzzzzzz");
    if (are_positions_equal()) {
        // RCLCPP_INFO(this->get_logger(), "xxxxxx");
        return;
    }
    ex_x_ = odometry_.pose.pose.position.x;
    ex_y_ = odometry_.pose.pose.position.y;
    
    // 回避を開始する点を取得
    auto odom_position = odometry_.pose.pose.position;
    double previous_distance = std::numeric_limits<double>::infinity();
    int current_index = 0;
    for (const auto & point : path_.poses) {
        auto path_position = point.pose.position;

        double distance = std::sqrt(
            std::pow(odom_position.x - path_position.x, 2) +
            std::pow(odom_position.y - path_position.y, 2)
        );

        // if (current_index < 10) {
        //     RCLCPP_INFO(this->get_logger(), "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
        //     RCLCPP_INFO(this->get_logger(), "Point x: %f, y: %f, distance: %f", path_position.x, path_position.y, distance);
        // }

        if (distance > previous_distance) {
            current_index -= 1;
            break;
        }

        previous_distance = distance;
        current_index++;
    }
    int start_index = current_index + start_index_;
    current_index = start_index;
    // RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance current index: %ld", current_index);

    generate_paths_.clear();
    generate_paths_.push_back(path_.poses[start_index]);

    for (int i = 0; i < num_change_points_; i ++) {
        // 現在の位置と姿勢を取得
        double x = generate_paths_[i].pose.position.x;
        double y = generate_paths_[i].pose.position.y;
        double theta = tf2::getYaw(generate_paths_[i].pose.orientation);

        // 車両が目標とする地点を求める
        for (int j = current_index; j < path_.poses.size(); j++ ) {
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
        // RCLCPP_INFO(this->get_logger(), "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk");
        bool flag = false;
        for (const double angle : angles) {
            double angle_rad = (angle * M_PI) / 180.0;
            double new_x = x + desired_dist_ * cos(theta + angle_rad);
            double new_y = y + desired_dist_ * sin(theta + angle_rad);

            double attract = compute_attractive_potential(new_x, new_y, goal_x, goal_y);
            double repulse = compute_repulsive_potential(new_x, new_y, theta + angle_rad);
            double potential = attract_ * attract + repulse_ * repulse;

            // if (repulse != 0) {
            //     flag = true;
            //     RCLCPP_INFO(this->get_logger(), "Attract: %f, Repulse: %f", attract, repulse);
            // }

            potentials.push_back(potential);
        }

        // if (flag) {
        //     RCLCPP_INFO(this->get_logger(), "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk");
        // };

        int max_index = std::distance(potentials.begin(), std::max_element(potentials.begin(), potentials.end()));
        double angle_rad = (angles[max_index] * M_PI) / 180.0;

        std::vector<double> orientation;
        euler_to_quaternion(0, 0, angle_rad + theta, orientation);

        // TODO: 消す
        // double sample_x = x + desired_dist_ * cos(theta + angle_rad);
        // double sample_y = y + desired_dist_ * sin(theta + angle_rad);
        // double sample_angel = angles[max_indexompute_attractive];

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

    for (int i = start_index; i < start_index + margin_; i++) {
        double dist = std::sqrt(
            std::pow(path_.poses[i].pose.position.x - x, 2) +
            std::pow(path_.poses[i].pose.position.y - y, 2));

        if (dist < min_dist) {
            min_dist = dist;
            min_goal_index = i;
        }
    }
    min_goal_index++;

    // RCLCPP_INFO(this->get_logger(), "=======%d=======", min_goal_index);

    nav_msgs::msg::Path new_path = nav_msgs::msg::Path();
    new_path.header = path_.header;

    // before path
    int count_a = 0;
    for (int i = 0; i < start_index; i++) {
        new_path.poses.push_back(path_.poses[i]);
        count_a++;
    }
    // generate path
    int count_b = 0;
    for (int i = 0; i < generate_paths_.size(); i++) {
        new_path.poses.push_back(generate_paths_[i]);
        count_b++;
    }
    // after path
    int count_c = 0;
    for (int i = min_goal_index; i < path_.poses.size(); i++) {
        new_path.poses.push_back(path_.poses[i]);
        count_c++;
    }

    // RCLCPP_INFO(this->get_logger(), "count_a: %d, count_b: %d, count_c: %d", count_a, count_b, count_c);

    avoidance_path_pub_->publish(new_path);

    counter_++;
}

double ObstacleAvoidance::compute_attractive_potential(double x, double y, double gx, double gy) {
    // ゴールとの距離が小さいほど評価が高い

    return -1 * std::sqrt(std::pow(x - gx, 2) + std::pow(y - gy, 2));
}

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

    bool flag = false;

    for (int i = x_start; i < x_end; i++) {
        for (int j = y_start; j < y_end; j++) {
            std::pair<int, int> rotated_point = rotatePoint(i, j, theta - 1.75, center_x, center_y);
            int new_x = rotated_point.first;
            int new_y = rotated_point.second;
            int index = new_y * width_ + new_x;

            if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_) {
                double dist = std::sqrt(std::pow((center_x - x) * resolution_, 2) + std::pow((center_y - y) * resolution_, 2));
                double weight = (dist > 0) ? 1.0 / dist : 0.0;
                int cost = costmap_.data[index];
                repulse -= cost * weight;
                // if (cost != 127 && cost != 0) {
                //     flag = true;
                //     RCLCPP_INFO(this->get_logger(), "jjjjjjjjjjjj");
                // }
                // if (cost <= 80 && cost != 0) {
                //     flag = true;
                //     RCLCPP_INFO(this->get_logger(), "jjjjjjjjjjjj");
                // }
                if (!(cost >= 101) && cost != 0) {
                    flag = true;
                    // RCLCPP_INFO(this->get_logger(), "jjjjjjjjjjjj");
                }
            }

            if (visual_ && (i == x_start || i == x_end - 1 || j == y_start || j == y_end - 1)) {
                costmap.data[index] = near_point_dist_;
            }
        }
    }

    if (!flag) {
        repulse = 0.0;
    }

    costmap_2d_pub_->publish(costmap);

    // if (repulse != 0.0) {
    //     RCLCPP_INFO(this->get_logger(), "repulse: %d", repulse);
    // }

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