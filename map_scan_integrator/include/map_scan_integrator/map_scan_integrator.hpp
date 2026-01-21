// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "map_scan_integrator/map.hpp"
#include "map_scan_integrator/scan.hpp"

namespace map_scan_integrator
{
class MapScanIntegrator : public rclcpp::Node
{
    public:
        MapScanIntegrator();
        ~MapScanIntegrator();
        void declare_param(void);
        void init_param(void);
        void init_pubsub(void);
        void cb_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
        void cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
        int get_node_freq(void);
        void main_loop(void);
        bool integrate_map_scan(sensor_msgs::msg::LaserScan & output_msg);
        float search_with_angle(Grid gs, Grid ge);
        
        // TransForm
        void init_tf(void);
        bool get_tf_pose( 
            const std::string & target_frame, 
            const std::string & source_frame, 
            geometry_msgs::msg::Pose2D & pose);

    private:
        // Parameters
        float search_th_;
        int node_freq_;
        std::string base_frame_id_, odom_frame_id_, scan_frame_id_;

        // Initialize flag
        bool recieve_scan_, recieve_map_, init_tf_;

        // Publishers, Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_lsa_map_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_integrated_scan_;

        // TransForm
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Message
        sensor_msgs::msg::LaserScan msg_scan_;

        // Map
        std::shared_ptr<Map> map_;
        std::shared_ptr<Scan> scan_;
};

} // namespace map_scan_integrator
