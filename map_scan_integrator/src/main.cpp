// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "map_scan_integrator/map_scan_integrator.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_scan_integrator::MapScanIntegrator>();
    rclcpp::Rate loop_rate(node->get_node_freq());
    while(rclcpp::ok()){
        node->main_loop();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}