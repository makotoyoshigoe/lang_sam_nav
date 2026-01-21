// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "map_scan_integrator/map_scan_integrator.hpp"

namespace map_scan_integrator{

MapScanIntegrator::MapScanIntegrator()
: Node("map_scan_integrator")
{
    init_pubsub();
    declare_param();
    init_param();
    map_.reset(new Map());
    scan_.reset(new Scan());
}

void MapScanIntegrator::main_loop(void)
{
    if(!init_tf_) init_tf();
    if(!recieve_map_ || !recieve_scan_) return;
    sensor_msgs::msg::LaserScan output_msg;
    if(!integrate_map_scan(output_msg)) return;
    pub_integrated_scan_->publish(output_msg);
    // RCLCPP_INFO(get_logger(), "Publish");
}

bool MapScanIntegrator::integrate_map_scan(
    sensor_msgs::msg::LaserScan & output_msg)
{
    // Convert scan data from lidar frame to robot frame
    float ang_i = scan_->angle_increment_;
    float ang_min = scan_->angle_min_;
    float ang_max = scan_->angle_max_;
    float rng_max = scan_->range_max_;
    size_t s = static_cast<size_t>((ang_max - ang_min) / ang_i) + 1;
    // RCLCPP_INFO(rclcpp::get_logger("integrator"), "integ scan size: %d", s);
    // std::vector<std::array<float, 4>> integrated_data(s, std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f});
    
    // Set integrated map scan msg
    output_msg = msg_scan_;
    output_msg.ranges.clear();
    output_msg.ranges.resize(s);

    geometry_msgs::msg::Pose2D odom_to_base, base_to_lidar;
    if(!get_tf_pose(base_frame_id_, odom_frame_id_, odom_to_base)) return false;
    if(!get_tf_pose(scan_frame_id_, base_frame_id_, base_to_lidar)) return false;

    // 探索の始点を計算
    float bx = odom_to_base.x, by = odom_to_base.y, bt = odom_to_base.theta;
    float lx = base_to_lidar.x, ly = base_to_lidar.y;
    float xs = bx + lx * cos(bt) - ly * sin(bt);
    float ys = by + lx * sin(bt) + ly * cos(bt);
    Grid gs = map_->point_to_grid(xs, ys);

    // RCLCPP_INFO(get_logger(), "Start: Searching");

    // for(float ang = ang_min; ang < ang_max; ang += ang_i){
    for(size_t i = 0; i < s; ++i){
        // スキャン関連の処理
        float ang = ang_min + ang_i * i;
        float scan_r = scan_->out_angle(ang) ? INFINITY : scan_->ranges_[scan_->rad_to_index(ang)];

        // マップ探索の処理
        float search_ang = ang + odom_to_base.theta + base_to_lidar.theta;
        float xe = xs + search_th_ * cos(search_ang), ye = ys + search_th_ * sin(search_ang);
        Grid ge = map_->point_to_grid(xe, ye);
        float map_r = search_with_angle(gs, ge);
        // RCLCPP_INFO(rclcpp::get_logger("integrator"), "Angle: %f, Search Result: %f", ang, map_r);
        // integrated_data[i] = std::array<float, 4>{ang, scan_r, 0.0f, 0.0f};
        float range = std::min(scan_r, map_r);
        // if(map_r == INFINITY) range = INFINITY;
        // float range = map_r;
        // integrated_data[i] = std::array<float, 4>{ang, range, 0.0f, 0.0f};
        output_msg.ranges[i] = range;
        // RCLCPP_INFO(get_logger(), "Range Index: %d", i);
    }
    // RCLCPP_INFO(get_logger(), "Finish Integration");
    return true;
}

float MapScanIntegrator::search_with_angle(Grid gs, Grid ge)
{
    Grid g_cur = gs;
    Grid d{std::abs(ge.x - gs.x), std::abs(ge.y - gs.y)};
    Grid s{(gs.x < ge.x) ? 1 : -1, (gs.y < ge.y) ? 1 : -1};
    
    int err = d.x - d.y;
    float result = INFINITY;

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

int MapScanIntegrator::get_node_freq(){return node_freq_;}

MapScanIntegrator::~MapScanIntegrator(){}

}
