# Install

## Dependencies

### OS

Preferably Ubuntu 20.04 with ROS Noetic installed

REF: [ROS Noetic requirements](https://wiki.ros.org/noetic/Installation)

### Required packages

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

### Packages by used hardware

#### Realsense camera

```bash
sudo apt-get -y install ros-noetic-realsense2-camera
```

#### Asus camera

```bash
sudo apt-get -y install ros-noetic-openni2-launch
```

### Joystick packages

#### XBox

Depends on how you use controller

##### REF [xboxdrv](https://manpages.ubuntu.com/manpages/jammy/man1/xboxdrv.1.html)

##### REF [ubuntu_xboxdrv](https://github.com/raelgc/ubuntu_xboxdrv)

##### REF [xone](https://github.com/medusalix/xone)

##### pi-bluetooth

If using Raspberry PI with Ubuntu to have working bluetooth.

```bash
sudo apt-get -y install pi-bluetooth
```

#### PS3

For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

<https://help.ubuntu.com/community/Sixaxis>

### Optional packages

#### Data compression

For data compression to reduce network bandwidth usage. Required for use of compressed topics.

```bash
sudo apt-get -y install ros-noetic-image-transport-plugins
```

#### [Speed up compilation](http://www.jamessjackson.com/gcc/ccache/distcc/compiling/c++/2017/07/25/ccache-and-distcc/)

```bash
sudo apt-get -y install ccache distcc
```

```bash
export DISTCC_HOSTS='localhost/4'

export CC="distcc gcc"
export CXX="distcc g++"
catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver
```

## Install of this repo

```bash
# create workspace
mkdir ws && cd ws
# clone repo to source folder
git clone git@github.com:KoMatrich/hexapod_ros.git src
# build repo
catkin build
# setup environment variables
cd src && source bash_scripts/source_robot.sh
```

Optionally to speed up compile time:

```bash
catkin build --env-cache
```

[NOTE: if environment is changing it can have unwanted side effects.](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#accelerated-building-with-environment-caching)

For further info look in `bash_scripts/source_robot.sh`

## Compiling recommendations

* please add these compiler optimizations after first build.

### Raspberry Pi2

```bash
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7
```

### ODROID XU3

```bash
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -pipe -march=armv7-a -mcpu=cortex-a9 -mfloat-abi=hard
```

### [General guide](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html#profile-cookbook)

## [Issues](isues.md)
