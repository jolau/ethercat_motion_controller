#!/bin/bash

# setcap does not support symbolic links, so a potential symbolic link has to be resolved first.
resolved_symlink=$(readlink -f ${1})

printf ${resolved_symlink}

# setcap using password
echo ${2} | sudo -S setcap cap_net_raw+ep ${resolved_symlink}

# Update the links and cache to the shared catkin libraries.
# See https://stackoverflow.com/questions/9843178/linux-capabilities-setcap-seems-to-disable-ld-library-path
sudo ldconfig /opt/ros/$ROS_DISTRO/lib

# launch the node
roslaunch varileg_lowlevel_controller varileg_lowlevel_controller.launch frequency:="${3}" motor_current:="${4}"