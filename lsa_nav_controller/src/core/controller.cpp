// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>

#include "lsa_nav_controller/core/controller.hpp"

namespace lsa_nav_controller
{


// Controller::Controller(
//     float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
//     float lin_acc_th, float lin_dec_th, float ang_acc_th, float ang_dec_th, 
//     float kp, float ki, float kd, float rate)
// : lin_max_vel_(lin_max_vel), lin_min_vel_(lin_min_vel), ang_max_vel_(ang_max_vel), ang_min_vel_(ang_min_vel), 
//   lin_acc_th_(lin_acc_th), lin_dec_th_(lin_dec_th), ang_acc_th_(ang_acc_th), ang_dec_th_(ang_dec_th), 
//   kp_(kp), ki_(ki), kd_(kd), rate_(rate), ei_(0.), pre_e_(0.), pre_cmd_{0., 0.}
Controller::Controller(
    float lin_max_vel, 
    float ang_max_vel, 
    float kp, 
    float front_search_angle_start, 
    float front_search_angle_end, 
    float side_search_angle_start, 
    float side_search_angle_end, 
    float obs_avoid_th, 
    float target_wall_dist
)
: lin_max_vel_(lin_max_vel), 
  ang_max_vel_(ang_max_vel), 
  kp_(kp), 
  front_search_angle_start_(front_search_angle_start), 
  front_search_angle_end_(front_search_angle_end), 
  side_search_angle_start_(side_search_angle_start), 
  side_search_angle_end_(side_search_angle_end), 
  obs_avoid_th_(obs_avoid_th), 
  target_wall_dist_(target_wall_dist), 
  action_state_("follow_wall")
{
    // ei_ = 0.;
    // init_ei_ = false;
    // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "ki: %f, ei: %.3f, rate: %f", ki_, ei_, rate_);
}

void Controller::set_scan(std::shared_ptr<Scan> & scan){scan_ = scan;}

std::array<float, 2> Controller::get_cmd_vel(void)
{
    // 前方の障害物との最小距離を取得
    // [0]: angle, [1]: range
    std::array<float, 2> min_front_laser = scan_->get_minimum_laser(front_search_angle_start_, front_search_angle_end_);

    std::array<float, 2> cur_cmd;
    float min_ang = min_front_laser[0];
    float min_range = min_front_laser[1];
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "min range: %f", min_range);
    if(min_range < obs_avoid_th_){
        RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Avoidance");
        // action_state_ = "obstacle_avoidance";
        cur_cmd[0] = lin_max_vel_ * (min_range - 0.3) / obs_avoid_th_;
        cur_cmd[1] = -ang_max_vel_;
    }else{
        // get side lateral average
        RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Wall Following");
        float side_ave 
            = scan_->get_side_lat_ave(side_search_angle_start_, side_search_angle_end_);
        cur_cmd = {lin_max_vel_, get_ang_vel(side_ave)};
    }
    cur_cmd[0] = std::clamp(cur_cmd[0], (float)0., lin_max_vel_);
    cur_cmd[1] = std::clamp(cur_cmd[1], -ang_max_vel_, ang_max_vel_);
    return cur_cmd;
    // if(action_state_ == "follow_wall"){
    //     
    // }else if(action_state_ == "obstacle_avoidance"){
    //     if(min_front_laser[1] < obs_avoid_th_){

    //     }else{

    //     }

    // }else{
    //     cur_cmd = {0.0, 0.0};
    // }

    // cur_cmd[0] = std::min(get_lin_vel(ave_front), lin_max_vel_);
    // cur_cmd[1] = std::min(get_ang_vel(ave_right, ave_left), ang_max_vel_);
    // cur_cmd[1] = std::min(get_ang_vel(ave_right, ave_left), ang_max_vel_);
    // cur_cmd[0] = std::max(std::min(get_lin_vel(ave_front), lin_max_vel_), lin_min_vel_);
    // cur_cmd[1] = std::max(std::min(get_ang_vel(ave_right, ave_left), ang_max_vel_), ang_min_vel_);
    // cur_cmd = smooth_vel(cur_cmd);
    //pre_cmd_ = cur_cmd;
}

// std::array<float, 2> Controller::smooth_vel(std::array<float, 2> raw_cmd)
// {
//     float diff_lin_vel = raw_cmd[0] - pre_cmd_[0];
//     float diff_ang_vel = raw_cmd[1] - pre_cmd_[1];
//     return {
//         diff_lin_vel > lin_acc_th_ * rate_ ? pre_cmd_[0] + lin_acc_th_ * rate_: raw_cmd[0], 
//         diff_ang_vel > ang_acc_th_ * rate_ ? pre_cmd_[1] + ang_acc_th_ * rate_: raw_cmd[1]
//     };
// }

float Controller::get_lin_vel(float ave_front)
{
    return lin_max_vel_;
}

float Controller::get_ang_vel(float side_ave)
{
    float error = target_wall_dist_ - side_ave;
    return -kp_ * error;
}

Controller::~Controller(){}
    
} // namespace lsa_nav_controller
