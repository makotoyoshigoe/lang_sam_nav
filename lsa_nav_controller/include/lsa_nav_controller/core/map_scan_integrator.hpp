// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lsa_nav_controller/sensor/scan.hpp"
#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
class MapScanIntegrator{
    public:
    MapScanIntegrator(float search_th);
    ~MapScanIntegrator();
    void set_data(
        std::shared_ptr<Scan> & scan, 
        std::shared_ptr<Map> & map, 
        geometry_msgs::msg::Pose2D & odom_to_base, 
        geometry_msgs::msg::Pose2D & base_to_lidar);
    std::vector<std::array<float, 4>> integrate_map_scan(void);
    std::vector<std::array<float, 4>> cvt_data_lidar_to_robot(
        std::shared_ptr<Scan> & scan,
        geometry_msgs::msg::Pose2D & base_to_lidar_pose);
    float search_with_angle(
        Grid & gs, 
        Grid & ge);
    void get_integrated_msg(sensor_msgs::msg::LaserScan & msg);
    
    private:
    float search_th_;

    std::shared_ptr<Scan> scan_;
    std::shared_ptr<Map> map_;
    geometry_msgs::msg::Pose2D odom_to_base_, base_to_lidar_;
    sensor_msgs::msg::LaserScan integrated_map_scan_;

};
}