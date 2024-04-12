
// ROS Hexapod Controller Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs

#include <gait.h>

static const double PI = atan(1.0) * 4.0;

namespace Gait{
    bool isIdValid(int gait_id){
        return gait_id >= ID::NONE && gait_id < ID::NUM_GAIT_STYLES;
    }

    bool isSet(int gait_id){
        return gait_id >= 0 && gait_id < ID::NUM_GAIT_STYLES;
    }

    bool isSet(ID gait){
        return gait >= 0 && gait < ID::NUM_GAIT_STYLES;
    }

    ID fromId(int gait_id){
        assert(isIdValid(gait_id));
        return static_cast<ID>(gait_id);
    }

    ID next(ID gait){
        if( !isSet(gait) ){
            assert(false);
            return Gait::ID::NONE;
        }

        return static_cast<ID>((gait + 1) % ID::NUM_GAIT_STYLES);
    }
}

//==============================================================================
//  Constructor: Initialize gait variables
//==============================================================================

GaitSequencer::GaitSequencer(void)
{
    int GAIT_STYLE_ID = -1;
    ros::param::get("CYCLE_LENGTH", CYCLE_LENGTH);
    ros::param::get("LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT);
    ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    ros::param::get("GAIT_STYLE_ID", GAIT_STYLE_ID);

    is_travelling_ = false;
    in_cycle_ = false;

    cycle_period_ = 25; //TODO CYCLE_LENGTH/2
    extra_gait_cycle_ = 1;

    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    Gait::ID GaitStyle = Gait::fromId(GAIT_STYLE_ID);
    if ( GaitStyle == Gait::ID::NONE ){
        GaitStyle = Gait::fromId(0);
    }

    setupGait(GaitStyle);
}

//=============================================================================
// Gait Sequence Change
//=============================================================================

void GaitSequencer::sequence_change(std::vector<int> &vec)
{
    for (size_t i = 0; i < vec.size(); i++)
{
    vec[i] += 1;
    vec[i] %= cycle_steps_;
    }
}

void GaitSequencer::setupGait(Gait::ID gait){
    switch (gait)
    {
    case Gait::ID::TRIPOD:
        cycle_steps_ = 2;
        cycle_leg_number_ = {1, 0, 1, 0, 1, 0};
        ROS_INFO("Switching to Tripod Gait");
        break;

    case Gait::ID::TETRAPOD:
        cycle_steps_ = 3;
        cycle_leg_number_ = {1, 0, 2, 0, 2, 1};
        ROS_INFO("Switching to Tetrapod Gait");
        break;

    case Gait::ID::WAVE:
        cycle_steps_ = 6;
        cycle_leg_number_ = {0, 1, 2, 3, 4, 5};
        ROS_INFO("Switching to Wave Gait");
        break;

    case Gait::ID::RIPPLE:
        cycle_steps_ = 6;
        cycle_leg_number_ = {0, 2, 4, 1, 3, 5};
        ROS_INFO("Switching to Ripple Gait");
        break;

    case Gait::ID::NUM_GAIT_STYLES:
        // wont actually happen
        __builtin_unreachable();
        break;

    default:
        ROS_ERROR("Invalid to switch to Gait Style: %d", gait);
        ros::shutdown();
        break;
    };

    active_gait_ = gait;
    next_gait = Gait::next(active_gait_);
}

//=============================================================================
// step calculation
//=============================================================================

void GaitSequencer::cyclePeriod(const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel)
{
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    // Calculate current velocities for this period of the gait
    // This factors in the sinusoid of the step for accurate odometry
    period_height = sin(cycle_period_ * PI / CYCLE_LENGTH);

    gait_vel->linear.x = ((PI * base.x) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->linear.y = ((-PI * base.y) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->angular.z = ((PI * base.theta) / CYCLE_LENGTH) * period_height * (1.0 / dt);

    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
        auto step_index = cycle_leg_number_[leg_index];
        switch (step_index)
        {
        case 0:
            if(is_travelling_ == false)
                break;
            // Lifts the leg and move it forward
            period_distance = cos(cycle_period_ * PI / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = base.x * period_distance;
            feet->foot[leg_index].position.y = base.y * period_distance;
            feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
            feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
            break;
        default:
            // Moves legs backward pushing the body forward
            auto n = cycle_steps_-1;
            period_distance = 1.0/n * (cos(cycle_period_ * PI / CYCLE_LENGTH) -1 -2*(step_index-1)) + 1;
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
            break;
        }
    }

    last_time_ = current_time_;
}

//=============================================================================
// Gait Sequencing
//=============================================================================

void GaitSequencer::gaitCycle(const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel)
{
    // Convert velocities into actual distance for gait/foot positions
    geometry_msgs::Pose2D base;
    base.x = cmd_vel.linear.x / PI * CYCLE_LENGTH;
    base.y = cmd_vel.linear.y / PI * CYCLE_LENGTH;
    base.theta = cmd_vel.angular.z / PI * CYCLE_LENGTH;

    // Low pass filter on the values to avoid jerky movements due to rapid value changes
    smooth_base_.x = base.x * 0.05 + (smooth_base_.x * (1.0 - 0.05));
    smooth_base_.y = base.y * 0.05 + (smooth_base_.y * (1.0 - 0.05));
    smooth_base_.theta = base.theta * 0.05 + (smooth_base_.theta * (1.0 - 0.05));

    // Check to see if we are actually travelling
    if ((std::abs(smooth_base_.y) > 0.001) ||           // 1 mm
        (std::abs(smooth_base_.x) > 0.001) ||           // 1 mm
        (std::abs(smooth_base_.theta) > 0.00436332313)) // 0.25 degree
    {
        is_travelling_ = true;
    }
    else
    {
        is_travelling_ = false;
        // Check to see if the legs are in a non rest state
        for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
        {
            if ((std::abs(feet->foot[leg_index].position.x) > 0.001) ||            // 1 mm
                (std::abs(feet->foot[leg_index].position.y) > 0.001) ||            // 1 mm
                (std::abs(feet->foot[leg_index].orientation.yaw) > 0.034906585) || // 2 degrees
                std::abs(feet->foot[leg_index].position.z) > 0.001)                // 1 mm
            {
                // If so calculate the rest of the cycle and add another complete cycle
                // This forces another cycle to allow all legs to set down after travel is stopped
                extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
                break;
            }
            else
            {
                extra_gait_cycle_ = 1;
            }
        }

        // countdown for in_cycle state
        if (extra_gait_cycle_ > 1)
        {
            extra_gait_cycle_--;
            in_cycle_ = !(extra_gait_cycle_ == 1);
        }
    }

    // If either is true we consider the gait active
    if (is_travelling_ == true || in_cycle_ == true)
    {
        cyclePeriod(smooth_base_, feet, gait_vel);
        cycle_period_++;
    }
    else
    {
        // Reset period to start just to be sure. ( It should be here anyway )
        cycle_period_ = 0;
    }

    if (is_travelling_ == false){
        if (switch_gait){
            switch_gait = false;
            setupGait(next_gait);
        }
    }

    // Loop cycle and switch the leg groupings for cycle
    if (cycle_period_ == CYCLE_LENGTH)
    {
        cycle_period_ = 0;
        sequence_change(cycle_leg_number_);
    }
}

void GaitSequencer::setGait(Gait::ID gait){
    if ( switch_gait ){
        ROS_INFO("Waiting for gait to finish for switch.");
    }else{
        ROS_INFO("Gait switch pulse received.");

        if(isSet(gait)){
            next_gait = gait;
            ROS_DEBUG("Switching to Gait Style: %d", gait);
        }else{
            ROS_DEBUG("Gait not set using next default gait");
        }

        switch_gait = true;
    }
}
