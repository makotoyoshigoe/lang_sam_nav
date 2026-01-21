// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <cmath>

#include "map_scan_integrator/scan.hpp"

namespace map_scan_integrator
{
Scan::Scan(void)
{
}

void Scan::set_scan_data(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    angle_min_ = scan_msg->angle_min;
    angle_max_ = scan_msg->angle_max;
    angle_increment_ = scan_msg->angle_increment;
    range_min_ = scan_msg->range_min;
    range_max_ = scan_msg->range_max;
    frame_id_ = scan_msg->header.frame_id;
    if(ranges_.size() != scan_msg->ranges.size()){
        ranges_.clear();
        ranges_.resize(scan_msg->ranges.size());
    }
    float range_min = scan_msg->range_min;
    float range_max = scan_msg->range_max;
    for(size_t i = 0; i < scan_msg->ranges.size(); ++i){
        ranges_[i] = range_min > scan_msg->ranges[i] || scan_msg->ranges[i] > range_max ? 
                      range_max : scan_msg->ranges[i];
    }
}

size_t Scan::rad_to_index(float rad)
{
    size_t idx = static_cast<size_t>((rad - angle_min_) / angle_increment_);
    return idx < 0 || idx >= ranges_.size() ? -1 : idx;
}

float Scan::index_to_rad(size_t i)
{
    return i * angle_increment_ + angle_min_;
}

bool Scan::out_angle(float angle)
{
    return angle_min_ > angle ||  angle > angle_max_;
}

Scan::~Scan(){}

} // namespace lsa_nav_controller