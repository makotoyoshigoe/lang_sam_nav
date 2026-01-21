// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.hpp>
#include <tf2/convert.hpp>

#include "map_scan_integrator/map_scan_integrator.hpp"

namespace map_scan_integrator{

void MapScanIntegrator::init_tf(void)
{
    tf_buffer_.reset();
    tf_listener_.reset();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface(),
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    init_tf_ = true;
    RCLCPP_INFO(get_logger(), "Initialized TF");
}

bool MapScanIntegrator::get_tf_pose( 
    const std::string & target_frame, 
    const std::string & source_frame, 
    geometry_msgs::msg::Pose2D & pose)
{
    geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = target_frame;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped tmp_pose;
	try {
		this->tf_buffer_->transform(ident, tmp_pose, source_frame);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	pose.x = tmp_pose.pose.position.x;
	pose.y = tmp_pose.pose.position.y;
	pose.theta = tf2::getYaw(tmp_pose.pose.orientation);
	return true;
}

}