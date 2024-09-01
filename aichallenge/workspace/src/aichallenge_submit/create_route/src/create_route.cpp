#include "create_route.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <utility>

namespace create_route
{

Create_route::Create_route() : Node("create_route"), get_path_(false), costmap_received_(false)
{
    RCLCPP_INFO(this->get_logger(), "============= create_route ==============");

    // pub, subの初期化
    this->declare_parameter("start_index", 1);
    start_index_ = this->get_parameter("start_index").as_int();

    this->declare_parameter("end_index", 1);
    end_index_ = this->get_parameter("end_index").as_int();

    this->declare_parameter("section_count", 100);
    section_count_ = this->get_parameter("section_count").as_int();

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

    path_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Create_route::send_path_request, this)
    );
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
    if (get_path_) {
        path_timer_->cancel();
        return;
    }
    // リクエストを作成
    auto request_section_path = std::make_shared<path_service::srv::GetPath::Request>();
    request_section_path->csv_path = std::to_string(section_count_);

    using ServiceResponseFuture = rclcpp::Client<path_service::srv::GetPath>::SharedFuture;

    // レスポンス処理
    auto response_section_path_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        section_path_ = result->path;

        if (section_path_.poses.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Path section received with %ld points", section_path_.poses.size());
            get_path_ = true;
        }
    };

    // 非同期リクエスト送信
    get_path_client_->async_send_request(request_section_path, response_section_path_callback);
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
    avoidance_path_ = section_path_;
    // デバッグ出力
    std::string indices_str;
    for (int idx : indices) {
        RCLCPP_INFO(this->get_logger(), "Selected indices : %i", idx);
    }
    
    // avoidance_path_.poses.erase(avoidance_path_.poses.begin() + indices[0], avoidance_path_.poses.begin() + indices[1] + 1);
    // std::vector<geometry_msgs::msg::PoseStamped> newElements;
    // std::vector<int> newData = {1, 2, 3, 4};
    // for (int data : newData) {
    //     geometry_msgs::msg::PoseStamped newPose;
    //     newPose.pose.position.x = data;
    //     newElements.push_back(newPose);
    // }
    // avoidance_path_.poses.insert(section_path_.poses.begin() + indices[0], newElements.begin(), newElements.end());
    avoidance_path_.header.stamp = this->now();
    avoidance_path_.header.frame_id = "map";

    response->path = avoidance_path_;
    RCLCPP_INFO(this->get_logger(), "Client Path size: %zu", avoidance_path_.poses.size());
    RCLCPP_INFO(this->get_logger(), "Obstacle Path Send");
}

//  sectionの中から障害物の座標に最も近い座標のindexを返す
int Create_route::find_nearest_point_index(double x,double y) {
    double min_distance = std::numeric_limits<double>::infinity();
    int nearest_index = -1;

    for (size_t i = 0; i < section_path_.poses.size(); i++) {
        double distance = std::sqrt(
            std::pow(section_path_.poses[i].pose.position.x - x, 2) +
            std::pow(section_path_.poses[i].pose.position.x - y, 2)
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

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<create_route::Create_route>());
    rclcpp::shutdown();
    return 0;
}
