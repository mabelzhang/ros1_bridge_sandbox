#!/bin/bash

alias setup_ws=". /opt/ros/melodic/setup.bash \
 && . /opt/ros/dashing/setup.bash \
 && . $HOME/ros1_bridge_sandbox/ros1_msgs_ws/install_isolated/setup.bash \
 && . $HOME/ros1_bridge_sandbox/ros2_msgs_ws/install/local_setup.bash \
 && . $HOME/ros1_bridge_sandbox/bridge_ws/install/local_setup.bash \
 && ros2 run ros1_bridge dynamic_bridge --print-pairs \
 && ros2 run ros1_bridge dynamic_bridge --print-pairs | grep bridge"

