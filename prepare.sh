#ros
echo "soucing ros noetic"
source /opt/ros/noetic/setup.bash
echo "sourcing catkin_ws"
source ~/ws/devel/setup.bash

#mdns
export CONTROL=$(avahi-resolve -n ros-vm.local | awk '{print $2}')
export ROBOT=$(avahi-resolve -n ros.local | awk '{print $2}')

#=========================
# PC SPECIFIC !!!
#=========================
export ROS_IP=$CONTROL
#export ROS_HOSTNAME=ros-vm.local # dynamic but very slow
#=========================
# https://github.com/ms-iot/ros_comm/pull/56
#=========================

if true; then
    export MAIN_NODE=$ROBOT
else
    export MAIN_NODE=$CONTROL
fi

echo "Robot   IP:$ROBOT"
echo "Control IP:$CONTROL"
echo ""
echo "Roscore IP:$MAIN_NODE"

#ros
export ROS_MASTER_URI="http://$MAIN_NODE:11311/"

if true; then
   echo "Sync clock to robot"
   sudo service ntp stop
   sudo ntpdate $ROBOT
else
   echo "Skipping synchronization of clock to robot"
fi
