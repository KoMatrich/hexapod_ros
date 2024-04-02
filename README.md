
# WIP! (some features might be broken)

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

* Asus Xtion
* ~~Primesense Sensor or Intel Realsense~~
* ~~IMU (Current master branch uses a Phidgets 3/3/3 Spatial in launch files.)~~

## 3. Dependencies

### Main

```bash
sudo apt-get -y install git
sudo apt-get -y install ros-noetic-sound-play 
sudo apt-get -y install ros-noetic-diagnostic-updater 
sudo apt-get -y install ros-noetic-xacro
sudo apt-get -y install ros-noetic-depthimage-to-laserscan
sudo apt-get -y install ros-noetic-joystick-drivers
sudo apt-get -y install ros-noetic-imu-filter-madgwick
sudo apt-get -y install ros-noetic-robot-localization
sudo apt-get -y install ros-noetic-rtabmap
sudo apt-get -y install ros-noetic-rtabmap-ros
sudo apt-get -y install ros-noetic-robot-state-publisher
sudo apt-get -y install ros-noetic-gazebo-ros-control
sudo apt-get -y install ros-noetic-navigation
sudo apt-get -y install ros-noetic-move_base
sudo apt-get -y install ros-noetic-navfn
sudo apt-get -y install ros-noetic-amcl
sudo apt-get -y install ros-noetic-teleop-twist-keyboard
sudo apt-get -y install libusb-1.0-0-dev
```

#### Optional

For data compression to reduce network bandwidth usage.

```bash
suod apt-get -y install ros-noetic-image-transport-plugins
```

### [Speed up compilation](http://www.jamessjackson.com/gcc/ccache/distcc/compiling/c++/2017/07/25/ccache-and-distcc/)

```bash
sudo apt-get -y install ccache distcc
```

```bash
export PATH=/usr/lib/ccache:$PATH

export CCACHE_PREFIX=distcc
export DISTCC_HOSTS='localhost/4'
export ROS_PARALLEL_JOBS='-j'$(distcc -j)'  -l'$(distcc -j)
```

### Realsence cammera

```bash
sudo apt-get -y install ros-noetic-realsense2-camera
```

### Asus camera

```bash
sudo apt-get -y install ros-noetic-openni2-launch
```

### Joystick

#### XBox

Depends on how you use controller

##### REF [xboxdrv](https://manpages.ubuntu.com/manpages/jammy/man1/xboxdrv.1.html)

##### REF [ubuntu_xboxdrv](https://github.com/raelgc/ubuntu_xboxdrv)

##### REF [xone](https://github.com/medusalix/xone)

##### pi-bluetooth

```
sudo apt-get -y install pi-bluetooth 
```

###### ubuntu server

* <https://forums.raspberrypi.com/viewtopic.php?t=304000>

* <https://forums.raspberrypi.com/viewtopic.php?t=207025>
* <https://forums.raspberrypi.com/viewtopic.php?t=242281>

###### [not detecting LE devices](https://askubuntu.com/a/1336059/1249998)

```bash
sudo btmgmt le on
```

###### [keeps disconnecting](https://askubuntu.com/questions/1250989/unable-to-connect-to-bluetooth-devices-org-bluez-error-connectionattemptfailed)

#### PS3

For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

<https://help.ubuntu.com/community/Sixaxis>

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

* please add these compiler optimizations after first build.

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
*Click on picture for redirect to YouTube video.*

Rviz recording of 3D mapping using RTABmap.

[![ScreenShot](http://img.youtube.com/vi/-3Ejgy1nFOg/0.jpg)]
(<https://www.youtube.com/watch?v=-3Ejgy1nFOg>)

Small video of Golem research platform and IMU testing.

[![ScreenShot](http://img.youtube.com/vi/IP-1HebkZnU/0.jpg)]
(<https://www.youtube.com/watch?v=IP-1HebkZnU>)

Renée Love's odometry test video using the phantomX.

[![ScreenShot](http://img.youtube.com/vi/VYBAM0MrvWI/0.jpg)]
(<https://www.youtube.com/watch?v=VYBAM0MrvWI>)
