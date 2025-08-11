# Source ROS 
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

uxrce() {
    ./Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
}

