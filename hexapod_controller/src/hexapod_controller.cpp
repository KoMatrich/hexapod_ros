
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
#include <parallel_executor.h>
#include <loop_control.h>

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

    std::vector<ServoDriver> servo_drivers;
    ParallelExecutor<ServoDriver> parallel_executor(servo_drivers);

    XmlRpc::XmlRpcValue SERVOS_DRIVERS;
    ros::param::get( "SERVOS_DRIVERS", SERVOS_DRIVERS );
    ROS_DEBUG("Number of servo drivers: %d", SERVOS_DRIVERS.size());

    for( XmlRpc::XmlRpcValue::iterator it = SERVOS_DRIVERS.begin(); it != SERVOS_DRIVERS.end(); it++ )
    {
        std::string port;
        int baudrate = -1;

        ros::param::get( "SERVOS_DRIVERS/" + static_cast<std::string>( it->first ) + "/port", port);
        ros::param::get( "SERVOS_DRIVERS/" + static_cast<std::string>( it->first ) + "/baudrate", baudrate);
        int driver_id = atoi(it->first.c_str());

        ROS_DEBUG("Servo driver port: %s with baudrate: %d and driver_id: %d", port.c_str(), baudrate, driver_id);

        servo_drivers.emplace_back(port.c_str(), baudrate, driver_id);
    }

    if( servo_drivers.size() == 0 )
    {
        ROS_ERROR("No servo drivers found. Exiting.");
        ros::shutdown();
    }

    // Establish initial leg positions for default pose in robot publisher
    gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );
    ik.calculateIK( control.feet_, control.body_, &control.legs_ );
    control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
    control.publishOdometry( control.gait_vel_ );
    control.publishTwist( control.gait_vel_ );

    ros::AsyncSpinner spinner( 2 ); // Using X threads
    spinner.start();

    LoopControl loopControl( control.MASTER_LOOP_RATE, "Controller");

    ROS_INFO("Hexapod Controller is now running.");
    while( ros::ok() )
    {
        if( control.gait_switch_pulse ){
            control.gait_switch_pulse = false;
            ROS_INFO("Gait switch pulse received.");

            if ( gait.switch_gait ){
                ROS_INFO("Waiting for gait to finish.");
            }else{
                ROS_INFO("Switching to next gait.");
                gait.switch_gait = true;
            }
        }

        // Start button on controller has been pressed stand up
        if( control.getHexActiveState() == true && control.getPrevHexActiveState() == false )
        {
            ROS_INFO("Hexapod standing up.");
            parallel_executor.execute( &ServoDriver::lockServos );

            while( control.body_.position.z < control.STANDING_BODY_HEIGHT && ros::ok() )
            {
                control.body_.position.z = control.body_.position.z + 0.001;

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );

                //broadcast over USB2AX
                parallel_executor.execute( &ServoDriver::transmitServoPositionsInter, control.joint_state_, true );

                loopControl.sleep();
            }

            ROS_INFO("Hexapod is now standing.");
            control.setPrevHexActiveState( true );
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

            parallel_executor.execute( &ServoDriver::getServoLoadsIterative, &control.joint_state_, 0 );

            // Commit new positions as well as jointStates
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );

            //broadcast over USB2AX
            parallel_executor.execute( &ServoDriver::transmitServoPositionsInter, control.joint_state_, true );

            // Set previous hex state of last loop so we know if we are shutting down on the next loop
            control.setPrevHexActiveState( true );
        }

        // Shutting down hex so let us do a gradual sit down and turn off torque
        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == true )
        {
            ROS_INFO("Hexapod sitting down.");
            const geometry_msgs::Twist zero_vel;
            while( control.body_.position.z > control.SITTING_BODY_HEIGHT  && ros::ok() )
            {
                control.body_.position.z = control.body_.position.z - 0.001;

                // Gait Sequencer called to make sure we are on all six feet
                gait.gaitCycle( zero_vel, &control.feet_, &control.gait_vel_ );

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );

                //broadcast over USB2AX
                parallel_executor.execute( &ServoDriver::transmitServoPositionsInter, control.joint_state_, true );

                loopControl.sleep();
            }

            ROS_INFO("Hexapod is now sitting.");
            parallel_executor.execute( &ServoDriver::freeServos );

            // Locomotion is now shut off
            control.setPrevHexActiveState( false );
        }

        // Sitting down with servo torque off. Publish jointState message
        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == false )
        {
            // Commit new positions as well as jointStates
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );
        }

        loopControl.sleep();
    }
    ROS_INFO("Hexapod Controller is now shutting down.");
    return 0;
}


