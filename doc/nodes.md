# Nodes

## hexapod_controller

This is the main node of the stack. It handles all control, gait, IK and servo communications with the legs. Minimal latency was required to keep the gait smooth and synced with odometry hence the reason they are all combined in this one node.

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist) Velocity command.
     body_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the orientation of the body.
     head_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the pan and tilt of the optional turret.
     state (std_msgs::Bool) Bool array to record state of the hexapod. Standing up, sitting etc.
     imu/data (sensor_msgs::Imu) Used in optional auto body leveling on non level ground.

*Published Topics*

    sounds (hexapod_msgs::Sounds) Custom message to send sound cues to the optional sound package.
    joint_states (sensor_msgs::JointState) Joint states for rviz and such.
    odometry/calculated (nav_msgs::Odometry) Calculated odometry from the gait system in the package.
    twist (geometry_msgs::TwistWithCovarianceStamped) Twist message syncronized with the gait system.

## hexapod_gui

This package acts as control GUI node for this robot. Displays load information from Dynamixel servos. Can be used as controller.

*Subscribed Topics*

    joint_states (sensor_msgs::JointState) Used for getting load info

*Published Topics*

    joy (sensor_msgs::Joy) Output of pressed buttons
    next_gait_topic (std_msgs::String) Selected gait

## hexapod_teleop_joystick

This package remaps `\joy` messages to hexapod commands that are processed by [main node](#hexapod_controller).

## hexapod_bringup

This package has all the launch files. From simple locomotion only to full mapping and localization examples.

## hexapod_description

This package has all the param files. You will start with one of the param config files to describe your hexapod. It also has params for different telop controllers. The xacro and meshes also reside in this package.
