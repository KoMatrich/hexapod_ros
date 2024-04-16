#include "load_balancer.h"

LoadBalancer::LoadBalancer(double max_effort_cap, double max_displacement_cap):
    MAX_EFFORT_CAP(max_effort_cap), MAX_DISPLACEMENT_CAP(max_displacement_cap)
{
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
    ros::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_SEGMENTS );
}

void LoadBalancer::compensate(hexapod_msgs::FeetPositions &feet, const sensor_msgs::JointState &joint_state)
{
    for ( int leg_i = 0; leg_i < NUMBER_OF_LEGS; leg_i++ ){
        // compensate only legs that should be on ground
        if( feet.foot[leg_i].position.z > 0 )
            continue;

        // collect effort from all segments exept the first one
        double effort = 0;
        for( int joint_i = 1; joint_i < NUMBER_OF_LEG_SEGMENTS; joint_i++ ){
            auto seg_i = leg_i*NUMBER_OF_LEG_SEGMENTS + joint_i;
            effort += joint_state.effort[seg_i]/joint_i;
        }

        // limit effort
        effort = min(effort, MAX_EFFORT_CAP);
        assert( effort <= MAX_EFFORT_CAP );

        // compensate only if effort is positive (holding leg up)
        if( effort <= 0)
            continue;
        assert( 0 <= effort );

        // calculate displacement
        double displacement = MAX_DISPLACEMENT_CAP * effort / MAX_EFFORT_CAP;

        // check if displacement is within limits
        assert(0 <= displacement && displacement <= MAX_DISPLACEMENT_CAP);

        feet.foot[leg_i].position.z -= displacement;
        ROS_DEBUG("Compensating leg %d by %.4f because of effort %.1f", leg_i, displacement, effort);
    }
}
