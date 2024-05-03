# WIP! (some features might be broken)

# ROS Hexapod Stack (ROS Noetic)

## Documentation

This is my implementation of a hexapod functioning in the ROS framework. Works mainly with 3dof hexapod. (4dof needs some work mainly in GUI) It is still very much a work in progress and I am still actively developing it. Uses [RTAB-map](https://wiki.ros.org/rtabmap) for mapping.

* Author: Kevin M. Ochs
* Contributor: Shubhankar Das
* Contributor: Renée Love
* Contributor: Konstantinos Chatzilygeroudis
* Contributor: Kurt Eckhardt
* Contributor: Romain Reignier
* Contributor: Martin Kocich

## Expected Hardware for mapping

* Asus Xtion
* ~~IMU~~ disabled in launch files

## [Installation](doc/install.md)

## [Nodes](doc/nodes.md)

**Example Launch Commands**

```bash
roslaunch hexapod_bringup hexapod_simple.launch # without mapping enabled
roslaunch hexapod_bringup hexapod_full.launch # with mapping enabled

roslaunch hexapod_navigation move_base.launch # navigation

rosrun hexapod_gui main_gui_node # gui node (requires roscore to be running)
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
