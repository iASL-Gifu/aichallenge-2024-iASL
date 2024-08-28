#include "costmap_2d.hpp"
#include <cmath>
#include <iomanip>  // std::setprecisionのために必要
#include <fstream>  // ofstreamのために必要

namespace costmap_server
{

CostMap2D::CostMap2D() : Node("costmap_2d"), get_map_(false)
{
    RCLCPP_INFO(this->get_logger(), "================== Cost Map =======================");

    this->declare_parameter("inflation_radius", 2.0);
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();

    this->declare_parameter("wall_width", 2.0);
    wall_width_ = this->get_parameter("wall_width").as_double();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    costmap_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "costmap_2d", qos
    );

    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aichallenge/objects", 1, 
        std::bind(&CostMap2D::object_callback, this, std::placeholders::_1)
    );

    get_map_client_ = this->create_client<nav_msgs::srv::GetMap>("/ogm/map_server/map");

    // サービスが利用できるまで待機
    while (!get_map_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // タイマーを使ってリクエストを定期的に送信
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CostMap2D::send_map_request, this)
    );
}

void CostMap2D::send_map_request()
{
    if (get_map_) {
        // マップが取得できたらタイマーを止める
        timer_->cancel();
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

            // map_.infoの情報を表示
            // RCLCPP_INFO(this->get_logger(), "=========================================");
            // RCLCPP_INFO(this->get_logger(), "Map Resolution: %f", map_.info.resolution);
            // RCLCPP_INFO(this->get_logger(), "Map Width: %u", map_.info.width);
            // RCLCPP_INFO(this->get_logger(), "Map Height: %u", map_.info.height);
            // RCLCPP_INFO(this->get_logger(), "Map Origin: (x: %f, y: %f, z: %f)", 
            //             map_.info.origin.position.x, 
            //             map_.info.origin.position.y, 
            //             map_.info.origin.position.z);
            // RCLCPP_INFO(this->get_logger(), "Map Orientation: (x: %f, y: %f, z: %f, w: %f)", 
            //             map_.info.origin.orientation.x, 
            //             map_.info.origin.orientation.y, 
            //             map_.info.origin.orientation.z, 
            //             map_.info.origin.orientation.w);

        }
    };

    auto future_result = get_map_client_->async_send_request(request, response_received_callback);
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
        double radius = data[i + 3];

        auto [x_start, x_end, y_start, y_end] = calculateIndex(center_x, center_y, radius);

        for (auto y = y_start; y < y_end; y++) {
            for (auto x = x_start; x < x_end; x++) {
                if (std::sqrt((x * resolution_ - center_x) * (x * resolution_ - center_x) +
                            (y * resolution_ - center_y) * (y * resolution_ - center_y)) < radius) {
                    int index = y * width_ + x;
                    array[index] = 100;  // 障害物セルの値を設定
                }
            }
        }
    }

    createInflationLayer(array, costmap);

    // コストマップをパブリッシュ
    costmap_2d_pub_->publish(costmap);
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
            if (obstacle_map[width_ * map_y + map_x ] == 100) {                
                calculateInflation(costmap, map_x, map_y);
            }
        }
    }
}


void CostMap2D::calculateInflation(const uint32_t & map_x, const uint32_t & map_y)
{
    auto center_x = map_x * resolution_;
    auto center_y = map_y * resolution_;
    auto [x_start, x_end, y_start, y_end] = calculateIndex(center_x, center_y, wall_width_);

    // 正方形の範囲を探索
    for (auto y = y_start; y < y_end; y++) {
        for (auto x = x_start; x < x_end; x++) {
            double distance = std::sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) * resolution_;
            if (distance < wall_width_) {
                double cost = (1.0 - (distance / wall_width_)) * 100.0;
                int index = y * width_ + x;
                map_.data[index] = std::max(map_.data[index], static_cast<int8_t>(cost));
            }
        }
    }
}

void CostMap2D::calculateInflation(
    nav_msgs::msg::OccupancyGrid & map, const uint32_t & map_x, const uint32_t & map_y)
{
    auto center_x = map_x * resolution_;
    auto center_y = map_y * resolution_;
    auto [x_start, x_end, y_start, y_end] = calculateIndex(center_x, center_y, inflation_radius_);

    // 正方形の範囲を探索
    for (auto y = y_start; y <= y_end; y++) {  // y_endを含むように修正
        for (auto x = x_start; x <= x_end; x++) {  // x_endを含むように修正
            double distance = std::sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) * resolution_;
            if (distance < inflation_radius_) {
                int index = y * width_ + x;
                double cost = (1.0 - (distance / inflation_radius_)) * 100.0;
                map.data[index] = std::max(map.data[index], static_cast<int8_t>(cost));
            }
        }
    }
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
