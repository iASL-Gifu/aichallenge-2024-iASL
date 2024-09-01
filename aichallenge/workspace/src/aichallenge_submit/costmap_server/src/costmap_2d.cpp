#include "costmap_2d.hpp"
#include <cmath>
#include <iomanip>  // std::setprecisionのために必要
#include <fstream>  // ofstreamのために必要
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace costmap_server
{

CostMap2D::CostMap2D() : Node("costmap_2d"), get_map_(false), get_path_centerline_(false)
{
    RCLCPP_INFO(this->get_logger(), "================== Cost Map =======================");

    this->declare_parameter("outside_cost", 2);
    outside_cost_ = this->get_parameter("outside_cost").as_int();

    this->declare_parameter("wall_inflation_radius", 2.0);
    wall_inflation_radius_ = this->get_parameter("wall_inflation_radius").as_double();

    this->declare_parameter("wall_min_cost", 2);
    wall_min_cost_ = this->get_parameter("wall_min_cost").as_int();

    this->declare_parameter("wall_max_cost", 2);
    wall_max_cost_ = this->get_parameter("wall_max_cost").as_int();

    this->declare_parameter("object_inside_cost", 2);
    object_inside_cost_ = this->get_parameter("object_inside_cost").as_int();

    this->declare_parameter("object_inside_radius", 2.0);
    object_inside_radius_ = this->get_parameter("object_inside_radius").as_double();

    this->declare_parameter("object_inflation_radius", 2.0);
    object_inflation_radius_ = this->get_parameter("object_inflation_radius").as_double();

    this->declare_parameter("object_min_cost", 2);
    object_min_cost_ = this->get_parameter("object_min_cost").as_int();

    this->declare_parameter("object_max_cost", 2);
    object_max_cost_ = this->get_parameter("object_max_cost").as_int();

    this->declare_parameter("object_long_side", 2.0);
    object_long_side_ = this->get_parameter("object_long_side").as_double();

    this->declare_parameter("object_short_side", 2.0);
    object_short_side_ = this->get_parameter("object_short_side").as_double();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    costmap_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "costmap_2d", qos
    );

    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aichallenge/objects", 1, 
        std::bind(&CostMap2D::object_callback, this, std::placeholders::_1)
    );

    get_map_client_ = this->create_client<nav_msgs::srv::GetMap>("/ogm/map_server/map");

    get_path_client_ = this->create_client<path_service::srv::GetPath>("/get_path");

    // サービスが利用できるまで待機
    while (!get_map_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "map client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for map service to appear...");
    }

    // タイマーを使ってリクエストを定期的に送信
    map_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CostMap2D::send_map_request, this)
    );

    while (!get_path_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "path client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for path service to appear...");
    }

    path_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CostMap2D::send_path_request, this)
    );

}

void CostMap2D::object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!get_map_) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet.");
        return;
    }

    nav_msgs::msg::OccupancyGrid costmap = map_; // コストマップを初期化
    std::vector<double> data = msg->data;
    std::vector<int8_t> array = std::vector<int8_t>(map_.data.size(), 0);

    for (size_t i = 0; i < data.size(); i += 4) {
        double center_x = data[i] - origin_x_;
        double center_y = data[i + 1] - origin_y_;

        int x_index = center_x / resolution_;
        int y_index = center_y / resolution_;

        int index = y_index * width_ + x_index;
        array[index] = object_inside_cost_;
    }

    createInflationLayer(array, costmap);

    // コストマップをパブリッシュ
    costmap_2d_pub_->publish(costmap);
}

void CostMap2D::send_map_request()
{
    if (get_map_) {
        // マップが取得できたらタイマーを止める
        map_timer_->cancel();
        return;
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    using ServiceResponseFuture = rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;

    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        map_ = result->map;

        if (map_.data.size() > 0) {
            get_map_ = true;

            resolution_ = map_.info.resolution;
            width_ = map_.info.width;
            height_ = map_.info.height;
            origin_x_ = map_.info.origin.position.x;
            origin_y_ = map_.info.origin.position.y;

            createInflationLayer();

            costmap_2d_pub_->publish(map_);

            RCLCPP_INFO(this->get_logger(), "____ Cost Map Map Client ____");
        }
    };

    auto future_result = get_map_client_->async_send_request(request, response_received_callback);
}

void CostMap2D::send_path_request()
{
    if (get_path_centerline_) {
        // 両方のパスが取得できたらタイマーを止める
        path_timer_->cancel();
        return;
    }

    // centerlineのリクエストを作成
    auto request_centerline = std::make_shared<path_service::srv::GetPath::Request>();
    request_centerline->csv_path = "centerline";

    using ServiceResponseFuture = rclcpp::Client<path_service::srv::GetPath>::SharedFuture;

    // centerlineのレスポンス処理
    auto response_centerline_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        centerline_path_ = result->path;

        if (centerline_path_.poses.size() > 0) {
            get_path_centerline_ = true;
            RCLCPP_INFO(this->get_logger(), "____ Cost Map Path Client ____");
            RCLCPP_INFO(this->get_logger(), "Path centerline received with %ld points", centerline_path_.poses.size());
        }
    };

    // 非同期リクエスト送信
    get_path_client_->async_send_request(request_centerline, response_centerline_callback);
}

void CostMap2D::createInflationLayer()
{
    std::vector<int8_t> copy_data = map_.data;
    // マップのすべてのセルを探索する
    for (uint32_t map_x = 0; map_x < width_; map_x++) {
        for (uint32_t map_y = 0; map_y < height_; map_y++) {
            // 物体を検知したときだけ処理をする
            if (copy_data[width_ * map_y + map_x ] == 100) {                
                calculateInflation(map_x, map_y);
            }
        }
    }
}

void CostMap2D::createInflationLayer(
    std::vector<int8_t> & obstacle_map, nav_msgs::msg::OccupancyGrid & costmap)
{
    // マップのすべてのセルを探索する
    for (uint32_t map_x = 0; map_x < width_; map_x++) {
        for (uint32_t map_y = 0; map_y < height_; map_y++) {
            // 物体を検知したときだけ処理をする
            if (obstacle_map[width_ * map_y + map_x ] == object_inside_cost_) {                
                calculateInflation(costmap, map_x, map_y);
            }
        }
    }
}

void CostMap2D::calculateInflation(const uint32_t & map_x, const uint32_t & map_y)
{
    auto center_x = map_x * resolution_;
    auto center_y = map_y * resolution_;
    auto [x_start, x_end, y_start, y_end] = calculateIndex(center_x, center_y, wall_inflation_radius_);

    // 正方形の範囲を探索
    for (uint32_t y = y_start; y < y_end; y++) {
        for (uint32_t x = x_start; x < x_end; x++) {
            int index = y * width_ + x;
            if (x == map_x && y == map_y) {
                map_.data[index] = outside_cost_;
            } else {
                double distance = std::sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) * resolution_;
                if (distance < wall_inflation_radius_) {
                    double cost = wall_min_cost_ + (wall_max_cost_ - wall_min_cost_) * (1.0 - (distance / wall_inflation_radius_));
                    map_.data[index] = std::max(map_.data[index], static_cast<int8_t>(cost));
                }
            }
        }
    }
}

void CostMap2D::calculateInflation(
    nav_msgs::msg::OccupancyGrid & map, const uint32_t & map_x, const uint32_t & map_y)
{
    auto center_x = map_x * resolution_;
    auto center_y = map_y * resolution_;
    int x_start = std::floor((center_x - object_short_side_) / resolution_); 
    int x_end = std::floor((center_x + object_short_side_) / resolution_); 
    int y_start = std::floor((center_y - object_long_side_) / resolution_); 
    int y_end = std::floor((center_y + object_long_side_) / resolution_); 

    if (x_start <= 0) x_start = 0;
    if (y_start <= 0) y_start = 0;
    if (x_end >= width_) x_end = width_ - 1;
    if (y_end >= height_) y_end = height_ - 1;

    double min_dist = std::numeric_limits<double>::infinity();
    int index = 0;
    for (int i = 0; i < centerline_path_.poses.size(); i++) {
        double dist = std::sqrt(
            std::pow(centerline_path_.poses[i].pose.position.x - (center_x + origin_x_), 2) +
            std::pow(centerline_path_.poses[i].pose.position.y - (center_y + origin_y_), 2)
        );

        if (dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }

    double theta = tf2::getYaw(centerline_path_.poses[index].pose.orientation);

    // 正方形の範囲を探索
    for (uint32_t y = y_start; y <= y_end; y++) {  // y_endを含むように修正
        for (uint32_t x = x_start; x <= x_end; x++) {  // x_endを含むように修正
            std::pair<int, int> rotated_point = rotatePoint(x, y, theta - (M_PI / 2), map_x, map_y);
            int new_x = rotated_point.first;
            int new_y = rotated_point.second;
            int index = new_y * width_ + new_x;

            if (x == map_x && y == map_y) {
                map.data[index] = object_inside_cost_;
            } else {
                double ellipse_center_x = map_x * resolution_;
                double ellipse_center_y = map_y * resolution_;
                double ellipse_major_axis = object_short_side_;
                double ellipse_minor_axis = object_long_side_;

                double dx = x * resolution_ - ellipse_center_x;
                double dy = y * resolution_ - ellipse_center_y;
                if ((dx * dx) / (ellipse_major_axis * ellipse_major_axis) + 
                    (dy * dy) / (ellipse_minor_axis * ellipse_minor_axis) <= 1.0) {
                    double distance = std::sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) * resolution_;
                    double cost = object_min_cost_ + (object_max_cost_ - object_min_cost_) * (1.0 - (distance / object_inflation_radius_));
                    map.data[index] = std::max(map.data[index], static_cast<int8_t>(cost));
                }
                // double distance = std::sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) * resolution_;
                // if (distance < object_inflation_radius_) {
                //     double cost = object_min_cost_ + (object_max_cost_ - object_min_cost_) * (1.0 - (distance / object_inflation_radius_));
                //     map.data[index] = std::max(map.data[index], static_cast<int8_t>(cost));
                // }
            }
        }
    }
}

std::tuple<int, int, int, int> CostMap2D::calculateIndex(
    double x_m, double y_m, double radius)
{
    int x_start = std::floor((x_m - radius) / resolution_);
    int x_end = std::floor((x_m + radius) / resolution_);
    int y_start = std::floor((y_m - radius) / resolution_);
    int y_end = std::floor((y_m + radius) / resolution_);

    if (x_start <= 0) x_start = 0;
    if (y_start <= 0) y_start = 0;
    if (x_end >= width_) x_end = width_ - 1;
    if (y_end >= height_) y_end = height_ - 1;

    return {x_start, x_end, y_start, y_end};
}

std::pair<int, int> CostMap2D::rotatePoint(double x, double y, double theta, double center_x, double center_y)
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

//  平均0, 標準偏差がsigmaのガウス分布を作成して、確率変数がstochastic_variableのときの値を取得する
double CostMap2D::calculateCost(double stochastic_variable, double inflation_radius)
{
  double sigma = inflation_radius / 3.;
  double cost = 1. / std::sqrt(2. * M_PI * sigma * sigma) *
                std::exp(-stochastic_variable * stochastic_variable / (2. * sigma * sigma));
  return cost;
}

//  ガウス分布の最大値に対して、ある確率変数の値がどれだけの割合を持つかを計算する
double CostMap2D::normalizeCost(double max_pdf, double pdf) { return (pdf / max_pdf) * 100.; }

} // namespace costmap_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<costmap_server::CostMap2D>());
    rclcpp::shutdown();

    return 0;
}
