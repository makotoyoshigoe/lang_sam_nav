// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/sensor/scan.hpp"
#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
class WallFollowerController
{
    public:
    WallFollowerController();
    ~WallFollowerController();
    
    void set_scan(const Scan & scan);

    std::array<float , 2> get_cmd_vel(
        const geometry_msgs::msg::Pose2D & odom_to_base_pose,
        const geometry_msgs::msg::Pose2D & base_to_lidar_pose);
    
    private:
    std::unique_ptr<Scan> scan_;
    std::unique_ptr<Map> map_;
};
    
} // namespace lsa_nav_controller