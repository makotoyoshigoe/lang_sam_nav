// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <vector>

#include "lsa_nav_controller/sensor/scan.hpp"

namespace lsa_nav_controller
{
// struct std::array<float, 2>
// {
//     float lin_vel;
//     float ang_vel;
// };

class Controller{
    public:
    // Controller(
    //     float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
    //     float lin_acc_th, float lin_dec_th, float ang_acc_th, float ang_dec_th, 
    //     float kp, float ki, float kd, float rate);
    Controller(
        float lin_max_vel, 
        float ang_max_vel, 
        float kp,
        float front_search_angle_start, 
        float front_search_angle_end, 
        float side_search_angle_start,  
        float side_search_angle_end, 
        float obs_avoid_th, 
        float target_wall_dist
    );
    std::array<float, 2> get_cmd_vel(void);
    float get_ang_vel(float side_ave);
    float get_lin_vel(float ave_front);
    void set_scan(std::shared_ptr<Scan> & scan);
    // std::array<float, 2> smooth_vel(std::array<float, 2> raw_cmd);

    ~Controller();

    private:
    float lin_max_vel_, ang_max_vel_;
    float front_search_angle_start_, front_search_angle_end_;
    float side_search_angle_start_, side_search_angle_end_;
    float obs_avoid_th_, target_wall_dist_;
    //float lin_max_vel_, lin_min_vel_, ang_max_vel_, ang_min_vel_;
    //float lin_acc_th_, lin_dec_th_, ang_acc_th_, ang_dec_th_;
    //float kp_, ki_, kd_, rate_;
    float kp_;
    std::shared_ptr<Scan> scan_;
    std::string action_state_;
    // float ei_, pre_e_;
    // std::array<float, 2> pre_cmd_;
    // bool init_ei_;
};
    
} // namespace lsa_nav_controller
