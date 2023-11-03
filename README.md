

# ROS Hexapod Stackc (ROS Noetic)


## 1. Documentation

This is my implementation of a hexapod functioning in the ROS framework. Agnostic to either a 3dof or 4dof hexapod. It is still very much a work in progress and I am still actively developing it. 

Thanks to Shubhankar Das there are two gaits offered, the original sinusoidal tripod gait and a new ripple gait.

* Author: Kevin M. Ochs
* Contributor: Shubhankar Das
* Contributor: Renée Love
* Contributor: Konstantinos Chatzilygeroudis
* Contributor: Kurt Eckhardt
* Contributor: Romain Reignier
* Contributor: Martin Kocich

## 2. Expected Hardware for mapping

* Primesense Sensor, Asus Xtion or Intel Realsense
* ~~IMU (Current master branch uses a Phidgets 3/3/3 Spatial in launch files.)~~ Not i used

## 3. Dependencies

```
sudo apt-get install git
sudo apt-get install ros-noetic-sound-play-y 
sudo apt-get install ros-noetic-diagnostic-updater-y 
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-openni2-launch
sudo apt-get install ros-noetic-depthimage-to-laserscan
sudo apt-get install ros-noetic-joystick-drivers
sudo apt-get install ros-noetic-imu-filter-madgwick
sudo apt-get install ros-noetic-robot-localization
sudo apt-get install ros-noetic-rtabmap
sudo apt-get install ros-noetic-rtabmap-ros
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-move_base
sudo apt-get install ros-noetic-navfn
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt-get install libusb-1.0-0-dev
```

### Joystick

#### XBox
I tied these
##### REF [xboxdrv](https://manpages.ubuntu.com/manpages/jammy/man1/xboxdrv.1.html)
##### REF [ubuntu_xboxdrv](https://github.com/raelgc/ubuntu_xboxdrv)
```
sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
sudo apt-get update
sudo apt-get install ubuntu-xboxdrv
```
##### REF [xone](https://github.com/medusalix/xone)

##### pi-bluetooth
```
sudo apt-get install pi-bluetooth 
```
https://forums.raspberrypi.com/viewtopic.php?t=304000
https://forums.raspberrypi.com/viewtopic.php?t=207025
https://forums.raspberrypi.com/viewtopic.php?t=242281
#### PS3

For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

https://help.ubuntu.com/community/Sixaxis

## 4. Nodes

### hexapod_controller

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
     

### hexapod_bringup

This package has all the launch files. From simple locomotion only to full mapping and localization examples. 

### hexapod_description

This package has all the param files. You will start with one of the param config files to describe your hexapod. It also has params for different telop controllers. The xacro and meshes also reside in this package.


**Example Launch Command**
```
roslaunch hexapod_bringup hexapod_full.launch config:=phantomX joy_mapping:=joystick_ds3
```
## 5. Install

```
mkdir ws
cd ws
mkdir src
git clone git@github.com:KoMatrich/hexapod_ros.git src
catkin build
source devel/setup.bash
```

### Compiling recomandations
- please add these compiler optimizations after first build.
#### Raspberry Pi2
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7
```

#### ODROID XU3
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -pipe -march=armv7-a -mcpu=cortex-a9 -mfloat-abi=hard
```

## Videos 
------
_Click on picture for redirect to YouTube video._


Rviz recording of 3D mapping using RTABmap.

[![ScreenShot](http://img.youtube.com/vi/-3Ejgy1nFOg/0.jpg)]
(https://www.youtube.com/watch?v=-3Ejgy1nFOg)

Small video of Golem research platform and IMU testing.

[![ScreenShot](http://img.youtube.com/vi/IP-1HebkZnU/0.jpg)]
(https://www.youtube.com/watch?v=IP-1HebkZnU)

Renée Love's odometry test video using the phantomX.

[![ScreenShot](http://img.youtube.com/vi/VYBAM0MrvWI/0.jpg)]
(https://www.youtube.com/watch?v=VYBAM0MrvWI)


## Pictures

Rviz screenshot of point cloud and laserscan active.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/depthwithlaser.jpg)

2D room mapping in Rviz.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/2d_slam.jpg)

Renée Love's adaptation of the Hexapod stack for Trossen's  [PhantomX](http://www.trossenrobotics.com/phantomx-ax-hexapod.aspx).
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/1/2/6/6/9/screenshot_from_2015-04-22_20_23_15.png)

