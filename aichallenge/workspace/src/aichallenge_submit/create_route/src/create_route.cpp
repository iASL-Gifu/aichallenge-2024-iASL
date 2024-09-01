#include "create_route.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <utility>
#include <fstream>
#include <sstream>
#include <string>

namespace create_route
{

Create_route::Create_route() : Node("create_route"), get_path_(false), get_center_path_(false), costmap_received_(false)
{
    RCLCPP_INFO(this->get_logger(), "============= create_route ==============");

    // pub, subの初期化
    this->declare_parameter("start_index", 1);
    start_index_ = this->get_parameter("start_index").as_int();

    this->declare_parameter("end_index", 1);
    end_index_ = this->get_parameter("end_index").as_int();

    this->declare_parameter("section_count", 100);
    section_count_ = this->get_parameter("section_count").as_int();

    this->declare_parameter("left_csv_path", std::string("left_csv_path"));
    left_csv_path_ = this->get_parameter("left_csv_path").as_string();

    this->declare_parameter("right_csv_path", std::string("right_csv_path"));
    right_csv_path_ = this->get_parameter("right_csv_path").as_string();

    this->declare_parameter("margin_radius", 100.0);
    margin_radius_ = this->get_parameter("margin_radius").as_double();

    this->declare_parameter("angle_interval", 100.0);
    angle_interval_ = this->get_parameter("angle_interval").as_double();

    this->declare_parameter("collision_checker", 100.0);
    collision_checker_ = this->get_parameter("collision_checker").as_double();

    this->declare_parameter("left_start_index", 1);
    left_start_index_ = this->get_parameter("left_start_index").as_int();

    this->declare_parameter("left_end_index", 1);
    left_end_index_ = this->get_parameter("left_end_index").as_int();

    this->declare_parameter("right_start_index", 1);
    right_start_index_ = this->get_parameter("right_start_index").as_int();

    this->declare_parameter("right_end_index", 1);
    right_end_index_ = this->get_parameter("right_end_index").as_int();

    this->declare_parameter("center_start_index", 1);
    center_start_index_ = this->get_parameter("center_start_index").as_int();

    this->declare_parameter("center_end_index", 1);
    center_end_index_ = this->get_parameter("center_end_index").as_int();


    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "costmap_2d", 1,
        std::bind(&Create_route::costmap_callback, this, std::placeholders::_1));

    marker_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("create_route", 1);

    get_obstacle_path_srv_ = this->create_service<path_service::srv::GetObstaclePath>(
        "/obstacle_path_0", 
        std::bind(&Create_route::handle_get_obstacle_path, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_path_client_ = this->create_client<path_service::srv::GetPath>("/get_path");

    while (!get_path_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "path client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "wait for path service to appear...");
    }

    // タイマーの初期化
    delayed_timer_ = this->create_wall_timer(
        std::chrono::seconds(section_count_ * 3),  // 'x'を必要な秒数に変更
        static_cast<std::function<void()>>([this]() mutable {  // 'this' をキャプチャ
            // 指定の処理を実行
            path_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&Create_route::send_path_request, this)
            );

            load_csv(left_csv_path_, 1, left_path_);
            load_csv(right_csv_path_, 1, right_path_);

            // タイマーをキャンセルして再実行を防止
            this->delayed_timer_->cancel();
        })
    );

    RCLCPP_INFO(this->get_logger(), "Create Route %d xxxxxxxx", section_count_);
}

void Create_route::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = *msg;
    width_ = costmap_.info.width;
    height_ = costmap_.info.height;
    resolution_ = costmap_.info.resolution;
    origin_x_ = costmap_.info.origin.position.x;
    origin_y_ = costmap_.info.origin.position.y;
    costmap_received_ = true;

}

//sectionごとのpathをリクエストする関数
void Create_route::send_path_request()
{
    // 両方のパスが取得できているかを確認
    if (get_path_ && get_center_path_) {
        path_timer_->cancel();  // タイマーを停止
        return;
    }

    // リクエストを作成
    auto request_section_path = std::make_shared<path_service::srv::GetPath::Request>();
    request_section_path->csv_path = std::to_string(section_count_);

    using ServiceResponseFuture = rclcpp::Client<path_service::srv::GetPath>::SharedFuture;

    // レスポンス処理 (セクションパス)
    auto response_section_path_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        section_path_ = result->path;

        if (section_path_.poses.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Path section received with %ld points", section_path_.poses.size());
            get_path_ = true;  // セクションパスを取得
        }
    };

    // リクエストを作成（センターパス）
    auto request_center_path = std::make_shared<path_service::srv::GetPath::Request>();
    request_center_path->csv_path = "centerline";

    // レスポンス処理 (センターパス)
    auto response_center_path_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        center_path_ = result->path;

        if (center_path_.poses.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Path center received with %ld points", center_path_.poses.size());
            get_center_path_ = true;  // センターパスを取得
        }
    };

    // 非同期リクエスト送信
    if (!get_path_) {
        get_path_client_->async_send_request(request_section_path, response_section_path_callback);
    }

    if (!get_center_path_) {
        get_path_client_->async_send_request(request_center_path, response_center_path_callback);
    }
}


//障害物の座標が入ったリクエストを受け取ったとき、sectionのpathを変更して返す
void Create_route::handle_get_obstacle_path(const std::shared_ptr<path_service::srv::GetObstaclePath::Request> request,
std::shared_ptr<path_service::srv::GetObstaclePath::Response> response)
{
    double x = request->x;
    double y = request->y;

    RCLCPP_INFO(this->get_logger(), "Client Object x: %f, %f", x, y);

    int nearest_index = find_nearest_point_index(x, y);
    std::vector<int> indices = get_indices(nearest_index, start_index_, end_index_);
    // デバッグ出力
    std::string indices_str;
    for (int idx : indices) {
        RCLCPP_INFO(this->get_logger(), "Selected indices : %i", idx);
    }

    nav_msgs::msg::Path pathes = nav_msgs::msg::Path();
    pathes.header = section_path_.header;

    double race_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < section_path_.poses.size(); i++) {
        double dist = std::sqrt(
            std::pow(x - section_path_.poses[i].pose.position.x, 2) + 
            std::pow(y - section_path_.poses[i].pose.position.y, 2)
        );

        if (dist < race_dist) {
            race_dist = dist;
        }
    }

    if (race_dist > collision_checker_) {
        response->path = section_path_;
    } else {
        double left_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < left_path_.size(); i++) {
            double dist = std::sqrt(
                std::pow(x - left_path_[i].pose.position.x, 2) +
                std::pow(y - left_path_[i].pose.position.y, 2)
            );

            if (dist < left_dist) {
                left_dist = dist;
            }
        }

        double right_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < right_path_.size(); i++) {
            double dist = sqrt(
                std::pow(x - right_path_[i].pose.position.x, 2) +
                std::pow(y - right_path_[i].pose.position.y, 2)
            );

            if (dist < right_dist) {
                right_dist = dist;
            }
        }

        double center_dist = std::numeric_limits<double>::infinity();
        int index = 0;
        for (int i = 0; i < center_path_.poses.size(); i++) {
            double dist = sqrt(
                std::pow(x - center_path_.poses[i].pose.position.x, 2) +
                std::pow(y - center_path_.poses[i].pose.position.y, 2)
            );

            if (dist < center_dist) {
                index = i;
                center_dist = dist;
            }
        }

        double center_theta = tf2::getYaw(center_path_.poses[index].pose.orientation);

        RCLCPP_INFO(this->get_logger(), "left dist: %f, right dist: %f", left_dist, right_dist);

        std::vector<geometry_msgs::msg::PoseStamped> new_path;
        double theta_start = 0.0;
        double theta_end = 0.0;
        if (left_dist < right_dist) {
            theta_start = -180;
            theta_end = 0;

            for (double theta = theta_start; theta <= theta_end; theta += angle_interval_) {
                double new_x = x + margin_radius_ * cos(theta * (M_PI / 180.0) + center_theta);
                double new_y = y + margin_radius_ * sin(theta * (M_PI / 180.0) + center_theta);

                geometry_msgs::msg::PoseStamped point;
                point.pose.position.x = new_x;
                point.pose.position.y = new_y;
                point.pose.position.z = 43.1;

                new_path.push_back(point);
            }
        } else {
            theta_start = 180;
            theta_end = 0;

            for (double theta = theta_start; theta > theta_end; theta -= angle_interval_) {
                double new_x = x + margin_radius_ * cos(theta * (M_PI / 180.0) + center_theta);
                double new_y = y + margin_radius_ * sin(theta * (M_PI / 180.0) + center_theta);

                geometry_msgs::msg::PoseStamped point;
                point.pose.position.x = new_x;
                point.pose.position.y = new_y;
                point.pose.position.z = 43.1;

                new_path.push_back(point);
            }
        }

        for (int i = 0; i < indices[0]; i ++) {
            pathes.poses.push_back(section_path_.poses[i]);
        }

        for (int i = 0; i < new_path.size(); i++) {
            pathes.poses.push_back(new_path[i]);
        }

        for (int i = indices[1]; i < section_path_.poses.size() - indices[1]; i++) {
            pathes.poses.push_back(section_path_.poses[i]);
        }

        response->path = pathes;
    }
    
    RCLCPP_INFO(this->get_logger(), "Client Path size: %zu", response->path.poses.size());
    RCLCPP_INFO(this->get_logger(), "Obstacle Path Send");
}

//  sectionの中から障害物の座標に最も近い座標のindexを返す
int Create_route::find_nearest_point_index(double x,double y) {
    double min_distance = std::numeric_limits<double>::infinity();
    int nearest_index = -1;

    for (size_t i = 0; i < section_path_.poses.size(); i++) {
        double distance = std::sqrt(
            std::pow(section_path_.poses[i].pose.position.x - x, 2) +
            std::pow(section_path_.poses[i].pose.position.y - y, 2)
        );

        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Nearest waypoint index: %d, Distance: %f", nearest_index, min_distance);
    return nearest_index;
}

//障害物の座標に最も近いindexからrange点分前のindexと後ろのindexの入った配列を返す
std::vector<int> Create_route::get_indices(int index, int start, int end) {
    std::vector<int> indices;
    int start_index = std::max(0, index - start);
    int end_index = std::min(section_path_.poses.size() - 1, static_cast<size_t>(index + end));
    return {start_index, end_index};
}

void Create_route::euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result) {
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

void Create_route::load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point) {
    RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

    std::ifstream file(csv_path);
    std::string line;
    int line_count = 0;
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
    } else {
        RCLCPP_INFO(this->get_logger(), "Reading CSV file");
        std::getline(file, line);
        while (std::getline(file, line))
        {
            line_count++;
            
            if (line_count % downsample_rate != 0)
            {
                continue;
            }

            std::stringstream ss(line);
            std::string x, y, z;
            std::getline(ss, x, ',');
            std::getline(ss, y, ',');
            std::getline(ss, z, ',');

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = std::stof(x);
            pose.pose.position.y = std::stof(y);
            pose.pose.position.z = 43.1;

            pose.header.frame_id = "map";
            pose.header.stamp = this->now();

            point.push_back(pose);
        }
        file.close();
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points", point.size());
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<create_route::Create_route>());
    rclcpp::shutdown();
    return 0;
}
