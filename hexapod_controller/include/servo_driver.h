// ROS Hexapod Locomotion Node
// Copyright (c) 2014, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <memory>
#include <cmath>
#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <sensor_msgs/JointState.h>

//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================

using namespace std;

static constexpr auto PROTOCOL_VERSION  = 1.0;
static constexpr auto TORQUE_ON         = 1;
static constexpr auto TORQUE_OFF        = 0;
static constexpr auto LEN_GOAL_POSITION = 2;
static constexpr auto LEN_PRESENT_LOAD  = 2;

class ServoDriver
{
    public:
        ServoDriver( const char* device_name, uint baudrate=1000000, int driver_id=-1);
        ~ServoDriver( void );

        void transmitServoPositions( const sensor_msgs::JointState &joint_state );
        void transmitServoPositionsInter( const sensor_msgs::JointState &joint_state);

        void getServoLoadsIterative( sensor_msgs::JointState *joint_state, const uint LOAD_READ_EVERY );
        void getServoLoads( sensor_msgs::JointState *joint_state );
        void getServoLoad( sensor_msgs::JointState *joint_state, uint index );
        int getServoCount(){return SERVO_COUNT;};

        void makeSureServosAreOn();

        void freeServos( void );
        void lockServos( void );

        // Move constructor and move assignment operator
        ServoDriver(ServoDriver&& other) noexcept = default;
        ServoDriver& operator=(ServoDriver&& other) noexcept = default;

    private:
        void angleToRes( const sensor_msgs::JointState &joint_state );
        void resToAngle(       sensor_msgs::JointState &joint_state );

        unique_ptr<dynamixel::PortHandler> portHandler;
        unique_ptr<dynamixel::PacketHandler> packetHandler;

        uint8_t dxl_error = 0;                          // Dynamixel error
        uint16_t dxl_present_position = 0;              // Present position
        uint16_t currentPos, currentLoad;               // Current position, current load
        uint8_t param_goal_position[2];

        vector<float> cur_load_; // Current load of servos
        vector<int> cur_pos_;  // Current position of servos
        vector<int> goal_pos_; // Goal position of servos

        vector<int> pose_steps_; // Increment to use going from current position to goal position
        vector<int> write_pos_;  // Position of each servo for sync_write packet

        vector<double> OFFSET; // Physical hardware offset of servo horn
        vector<int> ID;        // Servo IDs
        vector<int> TICKS;     // Total number of ticks, meaning resolution of dynamixel servo
        vector<int> CENTER;    // Center value of dynamixel servo
        vector<double> MAX_RADIANS; // Max rotation your servo is manufactured to do. i.e. 360 degrees for MX etc.
        vector<double> RAD_TO_SERVO_RESOLUTION; // Radians to servo conversion
        vector<int> servo_orientation_; // If the servo is physically mounted backwards this sign is flipped

        vector<string> servo_map_key_; // Servo map key
        vector<int> servo_to_joint_index_; // Converts servo index to joint index

        bool portOpenSuccess = false;
        bool torque_on = true; //TODO: REFACTOR
        bool torque_off = true;
        bool writeParamSuccess = true;
        bool servos_free_;

        int DRIVER_ID;

        bool INTERPOLATION_LINEAR = true;
        int INTER_MAX_STEP_SIZE = 5;
        int INTER_MIN_STEP_SIZE = 1;

        uint SERVO_COUNT;
        int TORQUE_ENABLE, PRESENT_POSITION_L, PRESENT_LOAD_L, GOAL_POSITION_L, INTERPOLATION_LOOP_RATE;

        uint load_read_skipped; //number of cycles skipped
        uint current_index; //index of servo to read load from this cycle
};

// Check if the class is move constructible, move assignable
// need to be able to use emplace_back in to vector
// prevents big template errors when compiling
static_assert(std::is_move_assignable<ServoDriver>());
static_assert(std::is_move_constructible<ServoDriver>());

#endif // SERVO_DRIVER_H_
