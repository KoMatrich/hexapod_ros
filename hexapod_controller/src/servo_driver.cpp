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


#include <servo_driver.h>

//==============================================================================
//  Constructor: Open USB2AX and get parameters
// If servos are not on, no worries we read them later just to be safe
//==============================================================================

ServoDriver::ServoDriver( const char* device_name, uint baudrate, int driver_id)
    : portHandler(dynamixel::PortHandler::getPortHandler(device_name)),
    packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
    DRIVER_ID(driver_id)
{
    // Open port
    if ( portHandler.get()->openPort() )
    {
        ROS_INFO("Succeeded to open the port '%s' !", portHandler.get()->getPortName());
        auto currentBaudRate = portHandler.get()->getBaudRate();
        auto targetBaudRate = baudrate;
        // Set port baudrate
        if ( portHandler.get()->setBaudRate(baudrate) )
            ROS_INFO("Succeeded to change the baudrate! (%d) to (%d)", currentBaudRate, targetBaudRate);
        else
            ROS_WARN("Failed to change the baudrate! (%d) to (%d)", currentBaudRate, targetBaudRate);
        portOpenSuccess = true;
    }
    else
    {
        ROS_WARN("Failed to open the '%s', Ignore if using Rviz or Gazbebo", portHandler.get()->getPortName());
        ROS_WARN("Try changing permissions with \"sudo chown $USER %s\"", portHandler.get()->getPortName());
        return;
    }

    XmlRpc::XmlRpcValue SERVOS; // Servo map from yaml config file

    // Stating servos do not have torque applied
    ros::param::get( "TORQUE_ENABLE", TORQUE_ENABLE );
    ros::param::get( "PRESENT_POSITION_L", PRESENT_POSITION_L );
    ros::param::get( "PRESENT_LOAD_L", PRESENT_LOAD_L );
    ros::param::get( "GOAL_POSITION_L", GOAL_POSITION_L );
    ros::param::get( "SERVOS", SERVOS );
    ros::param::get( "INTERPOLATION_LOOP_RATE", INTERPOLATION_LOOP_RATE );

    ros::param::get( "INTERPOLATION_LINEAR", INTERPOLATION_LINEAR );
    ros::param::get( "INTER_MAX_STEP_SIZE", INTER_MAX_STEP_SIZE );
    ros::param::get( "INTER_MIN_STEP_SIZE", INTER_MIN_STEP_SIZE );

    int index = 0;
    for( XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++ )
    {
        int servo_driver_id = -1;
        ros::param::get( "SERVOS/" + static_cast<std::string>( it->first ) + "/driver_id", servo_driver_id );

        if( servo_driver_id == DRIVER_ID){
            servo_map_key_.push_back( it->first );
            servo_to_joint_index_.push_back( index );
        }else if( servo_driver_id == -1 ){
            ROS_WARN("No DRIVER_ID found for servo: %s", static_cast<std::string>( it->first ).c_str());
        }
        index++;
    }

    SERVO_COUNT = servo_map_key_.size();

    OFFSET.resize( SERVO_COUNT );
    ID.resize( SERVO_COUNT );
    TICKS.resize( SERVO_COUNT );
    CENTER.resize( SERVO_COUNT );
    MAX_RADIANS.resize( SERVO_COUNT );
    RAD_TO_SERVO_RESOLUTION.resize( SERVO_COUNT );
    servo_orientation_.resize( SERVO_COUNT );

    cur_load_.resize( SERVO_COUNT );
    cur_pos_.resize( SERVO_COUNT );
    goal_pos_.resize( SERVO_COUNT );

    write_pos_.resize( SERVO_COUNT );
    pose_steps_.resize( SERVO_COUNT );

    ROS_DEBUG("Adding %d servos to driver with id: %d", SERVO_COUNT, DRIVER_ID);
    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/offset", OFFSET[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/id", ID[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/ticks", TICKS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/center", CENTER[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/max_radians", MAX_RADIANS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/sign", servo_orientation_[i] );
        RAD_TO_SERVO_RESOLUTION[i] = TICKS[i] / MAX_RADIANS[i];
        // Fill vector containers with default value
        cur_pos_[i] = CENTER[i];
        goal_pos_[i] = CENTER[i];
        write_pos_[i] = CENTER[i];
        pose_steps_[i] = 1;
        ROS_DEBUG("Added servo ID: %d, Offset: %f, Ticks: %d, Center: %d, Max Radians: %f, Orientation: %d",
            ID[i], OFFSET[i], TICKS[i], CENTER[i], MAX_RADIANS[i], servo_orientation_[i]);
    }

    servos_free_ = true;
}

//==============================================================================
// Destructor: Turn off the torque of the servos then close the serial port
//==============================================================================

ServoDriver::~ServoDriver( void )
{
    // prevent destruction of nullptr when moving
    if( portHandler.get() == nullptr )
        return;

    ROS_INFO("Shutting down servo driver with id: %d", DRIVER_ID);
    freeServos();
    portHandler.get()->closePort();
}
//==============================================================================
// Convert angles to servo resolution each leg
//==============================================================================

void ServoDriver::angleToRes( const sensor_msgs::JointState &joint_state )
{
    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        int msg_i = servo_to_joint_index_[i];
        goal_pos_[i] = CENTER[i] + round( ( joint_state.position[msg_i] - ( servo_orientation_[i] * OFFSET[i] ) ) * RAD_TO_SERVO_RESOLUTION[i] );
    }
}

//==============================================================================
// Convert servo resolution to angles each leg
//==============================================================================

void ServoDriver::resToAngle( sensor_msgs::JointState &joint_state )
{
    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        int msg_i = servo_to_joint_index_[i];
        joint_state.position[msg_i] = ( cur_pos_[i] - CENTER[i] ) / RAD_TO_SERVO_RESOLUTION[i] + ( servo_orientation_[i] * OFFSET[i] );
    }
}

//==============================================================================
// Turn torque on and read current positions
//==============================================================================

void ServoDriver::makeSureServosAreOn()
{
    if( !servos_free_ || !portOpenSuccess )
    {
        // Servos are on so return
        return;
    }
    else
    {
        // Initialize current position as cur since values would be 0 for all servos ( Possibly servos are off till now )
        for( uint i = 0; i < SERVO_COUNT; i++ )
        {
            // Read present position
            if( packetHandler.get()->read2ByteTxRx(portHandler.get(), ID[i], PRESENT_POSITION_L, &currentPos, &dxl_error) == COMM_SUCCESS )
            {
                cur_pos_[i] = currentPos;
                //ROS_INFO("[ID:%02d]  PresPos:%02d", ID[i], cur_pos_[i]);
            }
            else
            {
                ROS_WARN("Read error on [ID:%02d]", ID[i]);
            }
        }
        lockServos();
    }
}

//==============================================================================
// Updates the positions of the servos and sends USB2AX broadcast packet (without interpolation)
//==============================================================================

void ServoDriver::transmitServoPositions( const sensor_msgs::JointState &joint_state )
{
    if( !portOpenSuccess )
        return;

    angleToRes( joint_state ); // Convert angles to servo resolution
    makeSureServosAreOn();

    dynamixel::GroupSyncWrite groupSyncWrite( portHandler.get(), packetHandler.get(), GOAL_POSITION_L, LEN_GOAL_POSITION );

    // Prepare packet for broadcast
    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        write_pos_[i] = goal_pos_[i];

        if( cur_pos_[i] == goal_pos_[i] )
        {
            continue;
        }

        // Add param goal position
        param_goal_position[0] = DXL_LOBYTE(write_pos_[i]);
        param_goal_position[1] = DXL_HIBYTE(write_pos_[i]);
        if( !groupSyncWrite.addParam(ID[i], param_goal_position) )
        {
            ROS_WARN("Goal position param write failed on [ID:%02d]", ID[i]);
            writeParamSuccess = false;
        }
    }

    // Broadcast packet over U2D2
    if( writeParamSuccess )
    {
        if( groupSyncWrite.txPacket() != COMM_SUCCESS ) ROS_WARN("Position write not successful!");
    }

    if( writeParamSuccess ){
        // Store write pose as current pose (goal) since we are now done
        for( uint i = 0; i < SERVO_COUNT; i++ )
        {
            cur_pos_[i] = write_pos_[i];
        }
    }
}


//==============================================================================
// Updates the positions of the servos and sends USB2AX broadcast packet (with interpolation)
//==============================================================================
void ServoDriver::transmitServoPositionsInter( const sensor_msgs::JointState &joint_state)
{
    if( !portOpenSuccess )
        return;

    angleToRes( joint_state ); // Convert angles to servo resolution
    makeSureServosAreOn();

    int interpolating = 0;
    int max_step_size = INTER_MIN_STEP_SIZE;

    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        // If any of these differ we need to indicate a new packet needs to be sent
        if( cur_pos_[i] != goal_pos_[i] )
        {
            interpolating++;
            if( INTERPOLATION_LINEAR ){
                pose_steps_[i] = std::max(std::abs(cur_pos_[i]-write_pos_[i]), 1);
            }else{
                pose_steps_[i] = 1;
            }
            write_pos_[i] = cur_pos_[i];
        }
        else
        {
            // Nothing is moving on this particular servo
            pose_steps_[i] = 0;
            write_pos_[i] = goal_pos_[i];
        }

        if( max_step_size < pose_steps_[i] ){
            max_step_size = pose_steps_[i];
        }
    }

    // If nothing moved we abort no need to send packet with same positions
    if( interpolating == 0 )
        return;

    if( INTERPOLATION_LINEAR ){
        int step_size = std::min(max_step_size,INTER_MAX_STEP_SIZE);

        for( uint i = 0; i < SERVO_COUNT; i++ )
        {
            pose_steps_[i] = std::ceil(pose_steps_[i] * step_size / (float)max_step_size);
        }
    }

    // Interpolation is needed
    ros::Rate loop_rate( INTERPOLATION_LOOP_RATE );
    dynamixel::GroupSyncWrite groupSyncWrite( portHandler.get(), packetHandler.get(), GOAL_POSITION_L, LEN_GOAL_POSITION );

    while( interpolating != 0 && ros::ok() )
    {
        // Prepare packet for broadcast
        for( uint i = 0; i < SERVO_COUNT; i++ )
        {
            if( pose_steps_[i] == 0 )
            {
                continue;
            }

            if( cur_pos_[i] < goal_pos_[i] )
            {
                write_pos_[i] = write_pos_[i] + pose_steps_[i];

                if( write_pos_[i] >= goal_pos_[i] )
                {
                    write_pos_[i] = goal_pos_[i];
                    pose_steps_[i] = 0;
                    interpolating--;
                }
            }

            if( cur_pos_[i] > goal_pos_[i] )
            {
                write_pos_[i] = write_pos_[i] - pose_steps_[i];

                if( write_pos_[i] <= goal_pos_[i] )
                {
                    write_pos_[i] = goal_pos_[i];
                    pose_steps_[i] = 0;
                    interpolating--;
                }
            }

            // Add param goal position
            param_goal_position[0] = DXL_LOBYTE(write_pos_[i]);
            param_goal_position[1] = DXL_HIBYTE(write_pos_[i]);
            if( !groupSyncWrite.addParam(ID[i], param_goal_position) )
            {
                ROS_WARN("Goal position param write failed on [ID:%02d]", ID[i]);
                writeParamSuccess = false;
            }
        }

        // Broadcast packet over U2D2
        if( writeParamSuccess )
        {
            if( groupSyncWrite.txPacket() != COMM_SUCCESS ) ROS_WARN("Position write not successful!");
        }
        groupSyncWrite.clearParam();

        loop_rate.sleep();
    }

    if( writeParamSuccess ){
        // Store write pose as current pose (goal) since we are now done
        for( uint i = 0; i < SERVO_COUNT; i++ )
        {
            cur_pos_[i] = write_pos_[i];
        }
    }
}

//==============================================================================
// Convert load data to a signed integer
//==============================================================================

float convertLoad(uint32_t data) {
    //REF: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-load-40

    // Mask for extracting the load direction (10. bit)
    constexpr uint32_t loadDirectionMask = 1 << 10;
    // Mask for extracting the load ratio (9. to 0. bits)
    constexpr uint32_t loadRatioMask = (1 << 10) -1;

    // Extract the load direction
    bool loadDirection = (data & loadDirectionMask) != 0;

    // Extract the load ratio
    int loadRatio = data & loadRatioMask;

    // Change sign if needed
    loadRatio = loadDirection ? -loadRatio : loadRatio;

    return loadRatio * 100 / 1024.0;
}

//==============================================================================
// Read the load of one servo each call
//==============================================================================

void ServoDriver::getServoLoadsIterative( sensor_msgs::JointState *joint_state, const uint LOAD_READ_EVERY)
{
    if( SERVO_COUNT == 0 )
        return;

    joint_state->header.stamp = ros::Time::now();

    // makes main loop run faster
    if( load_read_skipped++ >= LOAD_READ_EVERY ){
        load_read_skipped = 0;

        // read only one servo load per cycle to prevent large lag spikes
        current_index = (current_index+1)%SERVO_COUNT;
        getServoLoad( joint_state, current_index);
    }
}

//==============================================================================
// Read the load of each servo
//==============================================================================

void ServoDriver::getServoLoads( sensor_msgs::JointState *joint_state )
{
    joint_state->header.stamp = ros::Time::now();

    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        getServoLoad( joint_state, i);
    }
}

//==============================================================================
// Read the load of servo
//==============================================================================

void ServoDriver::getServoLoad( sensor_msgs::JointState *joint_state, uint index)
{
    assert( index < SERVO_COUNT );

    if( !portOpenSuccess )
        return;

    joint_state->header.stamp = ros::Time::now();

    if( packetHandler.get()->read2ByteTxRx(portHandler.get(), ID[index], PRESENT_LOAD_L, &currentLoad, &dxl_error) == COMM_SUCCESS )
    {
        cur_load_[index] = servo_orientation_[index] * convertLoad(currentLoad);

        int msg_i = servo_to_joint_index_[index];
        joint_state->effort[msg_i] = (cur_load_[index] + joint_state->effort[msg_i]*3)/4; // Remove noise from load data by averaging
    }
    else
    {
        ROS_WARN("Read error on [ID:%02d]", ID[index]);
    }
}

//==============================================================================
// Turn torque off to all servos
//==============================================================================

void ServoDriver::freeServos( void )
{
    if( !portOpenSuccess )
        return;

    // Turn off torque
    for( uint i = 0; i < SERVO_COUNT; i++ )
    {
        if( packetHandler.get()->write1ByteTxRx(portHandler.get(), ID[i], TORQUE_ENABLE, TORQUE_OFF, &dxl_error) != COMM_SUCCESS )
        {
            ROS_WARN("TURN TORQUE OFF FAILED ON SERVO [ID:%02d]", ID[i]);
            torque_off = false;
        }
    }
    if( torque_off )
    {
        ROS_INFO("Hexapod servos torque is now OFF.");
        servos_free_ = true;
    }
}

//==============================================================================
// Turn torque on to all servos
//==============================================================================

void ServoDriver::lockServos( void )
{
    if( !portOpenSuccess )
        return;

    // Turn on torque
    for( uint i = 0; i < SERVO_COUNT; i++ ){
        if( packetHandler.get()->write1ByteTxRx(portHandler.get(), ID[i], TORQUE_ENABLE, TORQUE_ON, &dxl_error) != COMM_SUCCESS )
        {
            ROS_WARN("TURN TORQUE ON SERVO FAILED [ID:%02d]", ID[i]);
            torque_on = false;
        }
    }
    if( torque_on )
    {
        ROS_INFO("Hexapod servos torque is now ON.");
        servos_free_ = false;
    }
}
