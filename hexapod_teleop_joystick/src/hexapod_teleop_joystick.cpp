
// ROS Hexapod Teleop Joystick Node
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


#include <hexapod_teleop_joystick.h>

//==============================================================================
// Constructor
//==============================================================================

HexapodTeleopJoystick::HexapodTeleopJoystick( void )
{
    state_.data = false; // Start in a standing state
    imu_override_.data = false;
    gait_switch_.data = false;
    NON_TELEOP = false; // Static but here for a safety precaution
    ros::param::get( "STANDUP_BUTTON", STANDUP_BUTTON );
    ros::param::get( "SITDOWN_BUTTON", SITDOWN_BUTTON );
    ros::param::get( "BODY_ROTATION_BUTTON", BODY_ROTATION_BUTTON );
    ros::param::get( "GAIT_SWITCH_BUTTON", GAIT_SWITCH_BUTTON );
    ros::param::get( "FORWARD_BACKWARD_AXES", FORWARD_BACKWARD_AXES );
    ros::param::get( "LEFT_RIGHT_AXES", LEFT_RIGHT_AXES );
    ros::param::get( "YAW_ROTATION_AXES", YAW_ROTATION_AXES );
    ros::param::get( "PITCH_ROTATION_AXES", PITCH_ROTATION_AXES );
    ros::param::get( "MAX_METERS_PER_SEC", MAX_METERS_PER_SEC );
    ros::param::get( "MAX_RADIANS_PER_SEC", MAX_RADIANS_PER_SEC );
    ros::param::get( "NON_TELEOP", NON_TELEOP );
    body_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/body_scalar", 100);
    head_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/head_scalar", 100);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    cmd_gait_switch_pub_ = nh_.advertise<std_msgs::Bool>("/cmd_gait_switch", 100);
    state_pub_ = nh_.advertise<std_msgs::Bool>("/state", 100);
    imu_override_pub_ = nh_.advertise<std_msgs::Bool>("/imu/imu_override", 100);

    last_joy_input_ = ros::Time::now();
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(DEFAULT_JOY_TOPIC, 5, &HexapodTeleopJoystick::joyCallback, this);
    joy_gui_sub = nh_.subscribe<sensor_msgs::Joy>(FALLBACK_JOY_TOPIC, 5, &HexapodTeleopJoystick::joyGuiCallback, this);
}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void HexapodTeleopJoystick::joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    updateJoy( joy, DEFAULT_JOY_TOPIC);
}

void HexapodTeleopJoystick::joyGuiCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    updateJoy( joy, FALLBACK_JOY_TOPIC);
}

void HexapodTeleopJoystick::updateJoy( const sensor_msgs::Joy::ConstPtr &joy, const char* topic){
    ros::Time current_time = ros::Time::now();

    if ( used_joy_topic_ == topic ){
        last_joy_input_ = current_time;
    }else{
        if ((current_time - last_joy_input_).toSec() < 5.0 ){
            return;
        }
        used_joy_topic_ = topic;
        ROS_INFO("Switching to available joy topic: %s", topic);
    }

    if( joy->buttons[STANDUP_BUTTON] == 1 )
    {
        if ( state_.data == false)
        {
            state_.data = true;
        }
    }

    if ( joy->buttons[SITDOWN_BUTTON] == 1 )
    {
        if ( state_.data == true)
        {
            state_.data = false;
        }
    }

    // Body rotation L1 Button for testing
    if( joy->buttons[BODY_ROTATION_BUTTON] == 1 )
    {
        imu_override_.data = true;
        body_scalar_.header.stamp = current_time;
        body_scalar_.accel.angular.x = -joy->axes[LEFT_RIGHT_AXES];
        body_scalar_.accel.angular.y = -joy->axes[FORWARD_BACKWARD_AXES];
        head_scalar_.header.stamp = current_time;
        head_scalar_.accel.angular.z = joy->axes[YAW_ROTATION_AXES];
        head_scalar_.accel.angular.y = joy->axes[PITCH_ROTATION_AXES];
    }
    else
    {
        imu_override_.data = false;
        cmd_vel_.linear.x = joy->axes[FORWARD_BACKWARD_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.linear.y = -joy->axes[LEFT_RIGHT_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.angular.z = joy->axes[YAW_ROTATION_AXES] * MAX_RADIANS_PER_SEC;
    }

    if( joy->buttons[GAIT_SWITCH_BUTTON] == 1 )
    {
        gait_switch_.data = true;
    }
    else
    {
        gait_switch_.data = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_teleop_joystick");
    HexapodTeleopJoystick hexapodTeleopJoystick;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop_rate( 100 ); // 100 hz
    while ( ros::ok() )
    {
        if( hexapodTeleopJoystick.NON_TELEOP == false ) // If True, assumes you are sending these from other packages
        {
            hexapodTeleopJoystick.cmd_vel_pub_.publish( hexapodTeleopJoystick.cmd_vel_ );
            hexapodTeleopJoystick.body_scalar_pub_.publish( hexapodTeleopJoystick.body_scalar_ );
            hexapodTeleopJoystick.head_scalar_pub_.publish( hexapodTeleopJoystick.head_scalar_ );
            hexapodTeleopJoystick.cmd_gait_switch_pub_.publish( hexapodTeleopJoystick.gait_switch_ );
        }
        hexapodTeleopJoystick.state_pub_.publish( hexapodTeleopJoystick.state_ ); // Always publish for means of an emergency shutdown type situation
        hexapodTeleopJoystick.imu_override_pub_.publish( hexapodTeleopJoystick.imu_override_ );
        loop_rate.sleep();
    }
}
