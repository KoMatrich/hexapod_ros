source ./bash_scripts/common_setup.sh

#=========================
# PC SPECIFIC !!!
#=========================
# static
export ROS_IP=$CONTROL
# dynamic but very slow
#export ROS_HOSTNAE=ros-vm.local
#=========================
# https://github.com/ms-iot/ros_comm/pull/56
#=========================
export SYNC_CLOCK=true
# export ROBOT=""
# ^ Uncoment to disable connection to robot
#=========================

if [ -z $ROBOT ]; then
   export CONNECTED_TO_ROBOT=false
   echo "Not connected to robot"
else
   export CONNECTED_TO_ROBOT=true
   echo "Connected to robot"
fi

if $SYNC_CLOCK && $CONNECTED_TO_ROBOT; then
   echo "Syncing clock with robot"
   sudo service ntp stop
   sudo ntpdate $ROBOT
else
   echo "Skipping synchronization of clock"
fi

if $CONNECTED_TO_ROBOT; then
    export MAIN_NODE=$ROBOT
    echo "Main node set to robot"
else
    export MAIN_NODE=$CONTROL
    echo "Running localy"
fi
export ROS_MASTER_URI="http://$MAIN_NODE:11311/"

echo "=============================="
echo "Roscore IP:$MAIN_NODE"
echo "=============================="

