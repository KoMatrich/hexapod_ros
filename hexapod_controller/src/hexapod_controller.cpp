
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


#include <ros/ros.h>
#include <control.h>
#include <gait.h>
#include <ik.h>
#include <servo_driver.h>

//=============================================================================
// Main
//=============================================================================

int main( int argc, char **argv )
{
    ros::init(argc, argv, "hexapod_controller");

    // Create class objects
    Control control;
    Gait gait;
    Ik ik;
    ServoDriver servoDriver;

    // Establish initial leg positions for default pose in robot publisher
    gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );
    ik.calculateIK( control.feet_, control.body_, &control.legs_ );
    control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
    control.publishOdometry( control.gait_vel_ );
    control.publishTwist( control.gait_vel_ );

    ros::Time current_time_, last_time_;

    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    ros::AsyncSpinner spinner( 2 ); // Using X threads
    spinner.start();
    ros::Rate loop_rate( control.MASTER_LOOP_RATE );  // Speed limit of loop ( Will go slower than this )

    ROS_INFO("Hexapod Controller is now running.");
    int n;
    while( ros::ok() )
    {
        if( control.gait_switch_pulse ){
            control.gait_switch_pulse = false;

            gait.switch_gait = true;
            ROS_INFO("Gait switch pulse received.");
        }

        // Start button on controller has been pressed stand up
        if( control.getHexActiveState() == true && control.getPrevHexActiveState() == false )
        {
            // Lock servos
            servoDriver.lockServos();
            ros::Duration( 0.5 ).sleep();

            ROS_INFO("Hexapod standing up.");
            while( control.body_.position.z < control.STANDING_BODY_HEIGHT && ros::ok() )
            {
                control.body_.position.z = control.body_.position.z + 0.001;

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                                servoDriver.transmitServoPositions( control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );

            }
            control.setPrevHexActiveState( true );
            ROS_INFO("Hexapod is now standing.");
        }

        // We are live and standing up
        if( control.getHexActiveState() == true && control.getPrevHexActiveState() == true )
        {
            // Divide cmd_vel by the loop rate to get appropriate velocities for gait period
            control.partitionCmd_vel( &control.cmd_vel_ );
            
            // Gait Sequencer
            gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );

            // IK solver for legs and body orientation
            ik.calculateIK( control.feet_, control.body_, &control.legs_ );

            // Commit new positions and broadcast over USB2AX as well as jointStates
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            
            int skip = 10; // prevents overloading the servo bus
            if( n++ >= skip ){
                servoDriver.getServoLoad( control.joint_state_ );
                n = 0;
            }
            
            servoDriver.transmitServoPositions( control.joint_state_ );

            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );

            // Set previous hex state of last loop so we know if we are shutting down on the next loop
            control.setPrevHexActiveState( true );
        }

        // Shutting down hex so let us do a gradual sit down and turn off torque
        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == true )
        {
            ROS_INFO("Hexapod sitting down.");
            const geometry_msgs::Twist zero_vel;
            while( control.body_.position.z > -0.1  && ros::ok() )
            {
                control.body_.position.z = control.body_.position.z - 0.001;

                // Gait Sequencer called to make sure we are on all six feet
                gait.gaitCycle( zero_vel, &control.feet_, &control.gait_vel_ );

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                servoDriver.transmitServoPositions( control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );
            }
            ROS_INFO("Hexapod is now sitting.");

            // Release torque
            ros::Duration( 0.5 ).sleep();
            servoDriver.freeServos();

            // Locomotion is now shut off
            control.setPrevHexActiveState( false );
        }
        
        // Sitting down with servo torque off. Publish jointState message every half second
        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == false )
        {
            ros::Duration( 0.5 ).sleep();
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );
        }
        
        loop_rate.sleep();
        last_time_ = current_time_;
    }
    ROS_INFO("Hexapod Controller is now shutting down.");
    return 0;
}


