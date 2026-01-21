// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "map_scan_integrator/map.hpp"

namespace map_scan_integrator
{

Map::Map(void)
{

}

void Map::set_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
    frame_id_ = map->header.frame_id;
    width_ = map->info.width;
    height_ = map->info.height;
    resolution_ = map->info.resolution;
    p_org_.x = map->info.origin.position.x;
    p_org_.y = map->info.origin.position.y;
    tf2::Quaternion q;
    tf2::convert(map->info.origin.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    p_org_.theta = yaw;
    data_.resize(width_, std::vector<int8_t>(height_, -1));
    cvt_1d_to_2d(const_cast<std::vector<int8_t>&>(map->data));
    init_map_receive_ = true;
}

void Map::get_map_msg(nav_msgs::msg::OccupancyGrid & output)
{
    output.header.frame_id = frame_id_;
    output.info.height = height_;
    output.info.origin.position.x = p_org_.x;
    output.info.origin.position.y = p_org_.y;
    output.info.origin.orientation = p_org_q_;
    output.info.resolution = resolution_;
    output.info.width = width_;
    cvt_2d_to_1d(output.data);
}

std::string Map::get_map_frame_id(void){return frame_id_;}

void Map::cvt_2d_to_1d(std::vector<int8_t> & data)
{
    data.reserve(width_*height_);
    for(uint32_t i=0; i<height_; ++i){
        for(uint32_t j=0; j<width_; ++j){
            data.emplace_back(data_[j][i]);
        }
    }
}

void Map::cvt_1d_to_2d(std::vector<int8_t> & data)
{
    for(uint32_t y = 0; y < height_; ++y){
        for(uint32_t x = 0; x < width_; ++x){
            data_[x][y] = data[y*width_+ x];
        }
    }
}

bool Map::is_out_range(Grid grid)
{return grid.x < 0 || grid.x >= width_ || grid.y < 0 || grid.y >= height_;}

Grid Map::point_to_grid(float odom_x, float odom_y)
{   
    float diff_x = odom_x - p_org_.x, diff_y = odom_y - p_org_.y;
    float dx = diff_x * cos(p_org_.theta) + diff_y * sin(p_org_.theta);
    float dy = -diff_x * sin(p_org_.theta) + diff_y * cos(p_org_.theta);
    float x = p_org_.x + dx;
    float y = p_org_.y + dy;
    int ix = static_cast<int>((x - p_org_.x) / resolution_);
    int iy = static_cast<int>((y - p_org_.y) / resolution_);
    return {ix, iy};
}

void Map::set_odom_pose(geometry_msgs::msg::Pose2D odom_pose)
{odom_pose_ = odom_pose;}

float Map::get_dist_two_grids(Grid g1, Grid g2)
{
    Grid gd = g2 - g1;
    return hypot(gd.x, gd.y) * resolution_;
}

Map::~Map(){}

} // namespace lsa_nav_controller
