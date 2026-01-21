// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lsa_nav_controller/core/map_scan_integrator.hpp"

namespace lsa_nav_controller
{
MapScanIntegrator::MapScanIntegrator(float search_th)
: search_th_(search_th)
{
}

void MapScanIntegrator::set_data(
    std::shared_ptr<Scan> & scan,
    std::shared_ptr<Map> & map,
    geometry_msgs::msg::Pose2D & odom_to_base, 
    geometry_msgs::msg::Pose2D & base_to_lidar)
{
    scan_ = scan;
    map_ = map;
    odom_to_base_ = odom_to_base;
    base_to_lidar_ = base_to_lidar;
}

std::vector<std::array<float, 4>> MapScanIntegrator::integrate_map_scan(void)
{
    // Convert scan data from lidar frame to robot frame
    float ang_i = scan_->angle_increment_;
    //float ang_min = -M_PI;
    //float ang_max = M_PI;
    float ang_min = scan_->angle_min_;
    float ang_max = scan_->angle_max_;
    float rng_max = scan_->range_max_;
    size_t s = static_cast<size_t>((ang_max - ang_min) / ang_i) + 1;
    RCLCPP_INFO(rclcpp::get_logger("integrator"), "integ scan size: %d", s);
    std::vector<std::array<float, 4>> integrated_data(s, std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f});
    
    // Set integrated map scan msg
    integrated_map_scan_ = scan_->scan_msg_;
    integrated_map_scan_.angle_min = ang_min;
    integrated_map_scan_.angle_max = ang_max;
    integrated_map_scan_.ranges.clear();
    integrated_map_scan_.ranges.resize(s);

    // 探索の始点を計算
    float bx = odom_to_base_.x, by = odom_to_base_.y, bt = odom_to_base_.theta;
    float lx = base_to_lidar_.x, ly = base_to_lidar_.y;
    float xs = bx + lx * cos(bt) - ly * sin(bt);
    float ys = by + lx * sin(bt) + ly * cos(bt);
    Grid gs = map_->point_to_grid(xs, ys);

    // for(float ang = ang_min; ang < ang_max; ang += ang_i){
    for(size_t i = 0; i < s; ++i){
        // スキャン関連の処理
        float ang = ang_min + ang_i * i;
        float scan_r = scan_->out_angle(ang) ? rng_max : scan_->ranges_[scan_->rad_to_index(ang)];

        // マップ探索の処理
        float search_ang = ang + odom_to_base_.theta + base_to_lidar_.theta;
        float xe = xs + search_th_ * cos(search_ang), ye = ys + search_th_ * sin(search_ang);
        Grid ge = map_->point_to_grid(xe, ye);
        float map_r = search_with_angle(gs, ge);
        // RCLCPP_INFO(rclcpp::get_logger("integrator"), "Angle: %f, Search Result: %f", ang, map_r);
        // integrated_data[i] = std::array<float, 4>{ang, scan_r, 0.0f, 0.0f};
        float range = std::min(scan_r, map_r);
        integrated_data[i] = std::array<float, 4>{ang, range, 0.0f, 0.0f};
        integrated_map_scan_.ranges[i] = range;
    }
    return integrated_data;
}

float MapScanIntegrator::search_with_angle(
        Grid & gs, 
        Grid & ge)
{
    Grid g_cur = gs;
    Grid d{std::abs(ge.x - gs.x), std::abs(ge.y - gs.y)};
    Grid s{(gs.x < ge.x) ? 1 : -1, (gs.y < ge.y) ? 1 : -1};
    
    int err = d.x - d.y;
    float result = scan_->range_max_ * 1.1;

    while (true) {
        if (g_cur.x == ge.x && g_cur.y == ge.y) return result;
        if (map_->is_out_range(g_cur)) return result;
        if (map_->data_[g_cur.x][g_cur.y] == 100) return map_->get_dist_two_grids(gs, g_cur);
        int e2 = 2 * err;
        if (e2 > -d.y) {err -= d.y; g_cur.x += s.x;}
        if (e2 <  d.x) {err += d.x; g_cur.y += s.y;}
    }
    return result;
}

std::vector<std::array<float, 4>> MapScanIntegrator::cvt_data_lidar_to_robot(
    std::shared_ptr<Scan> & scan,
    geometry_msgs::msg::Pose2D & base_to_lidar_pose)
{
    std::vector<std::array<float, 4>> cvtd_scan_data;
    cvtd_scan_data.resize(scan->ranges_.size());
    float angle_min = scan->angle_min_;
    float angle_increment = scan->angle_increment_;
    float rx = base_to_lidar_pose.x, ry = base_to_lidar_pose.y, rt= base_to_lidar_pose.theta;
    for(size_t i = 0; i < scan->ranges_.size(); ++i){
        // Lidar座標系からロボット座標系への変換
        float range = scan->ranges_[i];
        float angle = angle_min + angle_increment * i + rt;
        float x = rx + range * std::cos(angle);
        float y = ry + range * std::sin(angle);
        cvtd_scan_data[i] = std::array<float, 4>{std::atan2(y, x), std::hypot(x, y), x, y};
        // 変換後の角度を標準出力
        // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Range: %.2f", cvtd_scan_data[i][1]);
    }
    return cvtd_scan_data;
}

void MapScanIntegrator::get_integrated_msg(
    sensor_msgs::msg::LaserScan & msg)
{
    msg = integrated_map_scan_;
}

MapScanIntegrator::~MapScanIntegrator(){}

}