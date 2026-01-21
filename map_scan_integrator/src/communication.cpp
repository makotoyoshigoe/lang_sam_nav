// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <map_scan_integrator/map_scan_integrator.hpp>

namespace map_scan_integrator{

void MapScanIntegrator::declare_param(void)
{
    declare_parameter("search_threshold", 10.0);
    declare_parameter("node_freq", 10);
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("odom_frame_id", "odom");
}

void MapScanIntegrator::init_param(void)
{
    search_th_ = get_parameter("search_threshold").as_double();
    node_freq_ = get_parameter("node_freq").as_int();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
}

void MapScanIntegrator::init_pubsub(void)
{
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&MapScanIntegrator::cb_scan, this, std::placeholders::_1));
    sub_lsa_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lang_sam_map", 10, std::bind(&MapScanIntegrator::cb_lsa_map, this, std::placeholders::_1));
    pub_integrated_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan/integrated", 10);
}

void MapScanIntegrator::cb_scan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    recieve_scan_ = true;
    scan_->set_scan_data(msg);
    scan_frame_id_ = msg->header.frame_id;
    msg_scan_ = *msg;
}

void MapScanIntegrator::cb_lsa_map(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    recieve_map_ = true;
    map_->set_lsa_map(msg);
}

}