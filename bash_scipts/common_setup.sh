echo "Running common part"
echo "=============================="

echo "Using ccache for fast compiling"
export PATH=/usr/lib/ccache:$PATH

echo "soucing ros noetic std libs"
source /opt/ros/noetic/setup.bash

if [ -f ../devel/setup.bash ]; then
    echo "Source dir is build"
else
    echo "Souce dir is not build"
    catkin build
fi

echo "sourcing catkin_ws"
source ../devel/setup.bash

echo "setting up MDNS"
export CONTROL=$(avahi-resolve -n ros-vm.local | awk '{print $2}')
export ROBOT=$(avahi-resolve -n ros.local | awk '{print $2}')

echo "=============================="
echo "Robot   IP:$ROBOT"
echo "Control IP:$CONTROL"
echo "Common part done"
echo "=============================="
