source ./bash_scripts/common_setup.sh

#=========================
# PC SPECIFIC !!!
#=========================
# static
export ROS_IP=$ROBOT
# dynamic but very slow
#export ROS_HOSTNAE=ros-vm.local
#=========================
# https://github.com/ms-iot/ros_comm/pull/56
#=========================

export MAIN_NODE=$ROBOT
export ROS_MASTER_URI="http://$MAIN_NODE:11311/"

echo "=============================="
echo "Roscore IP:$MAIN_NODE"
echo "=============================="

