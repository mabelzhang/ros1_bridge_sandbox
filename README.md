Stripped down packages to demonstrate the usage of [`ros1_bridge`](https://github.com/ros2/ros1_bridge), a communication channel between ROS 1 and ROS 2.

### Prerequisites

This package has been tested in Ubuntu 18.04 with
* ROS 1 Melodic
* ROS 2 Dashing


### Demo basic bridge usage

Shell 1, compile ROS 1 custom message:
```
. /opt/ros/melodic/setup.bash
cd ros1_bridge_sandbox
catkin_make_isolated --install
. install_isolated/setup.bash 
roscore &
```

Shell 2, compile ROS 2 custom message:
```
. /opt/ros/crystal/setup.bash
cd ros1_bridge_sandbox/ros2_msgs_ws
colcon build --packages-select bridge_msgs
. install/local_setup.bash
# Publishes bridge_msgs/JointCommand type to /joint_command topic
python3 src/bridge_msgs/src/ros2_pub.py
```

Shell 3, check out and compile `ros1_bridge` from source, to recognize the custom messages:
```
mkdir -p bridge_ws/src
cd bridge_ws/src
git clone https://github.com/ros2/ros1_bridge.git
cd ..
# Compile ros1_bridge
. /opt/ros/melodic/setup.bash
. /opt/ros/crystal/setup.bash
. ../ros1_bridge_sandbox/ros1_msgs_ws/install_isolated/setup.bash 
. ../ros1_bridge_sandbox/ros2_msgs_ws/install/local_setup.bash 
colcon build --packages-select ros1_bridge --cmake-force-configure
. install/local_setup.bash
# This should print:   - 'bridge_msgs/JointCommand' (ROS 2) <=> 'bridge_msgs/JointCommand' (ROS 1)
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep bridge
```

Shell 1, test subscribing to ROS 2 messages in ROS 1:
```
rosrun bridge_msgs ros1_sub.py
```

In the other direction, test subscribing to ROS 1 messages in ROS 2:
Shell 1:
```
rosrun bridge_msgs ros1_pub.py
```
Shell 2:
```
ros2 run bridge_msgs ros2_sub.py
```


### Demo bridge in robot simulation

Clone the [VRX simulation repository](https://bitbucket.org/osrf/vrx) to a desired location:
```
mkdir -p vrx_ws/src
cd vrx_ws/src
hg clone https://bitbucket.org/osrf/vrx
```

Compile and source the packages:
```
. /opt/ros/melodic/setup.bash
cd vrx_ws
catkin build
. devel/setup.bash
```

Launch VRX simulation in ROS 1:
```
roslaunch vrx_gazebo vrx.launch
```

Run the bridge:
```
. /opt/ros/melodic/setup.bash
. /opt/ros/dashing/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

Open a new terminal, source ROS 2, run RViz 2:
```
. /opt/ros/dashing/setup.bash
ros2 run rviz2 rviz2
```
Add Image topics and LIDAR topic to RViz. They should be readily displayed.

Run ROS 2 node to subscribe to built-in data types in ROS 1:
TODO: Test this
```
ros2 run bridge_msgs demo_vrx_read.py
```

Run ROS 2 node to publish to built-in data types in ROS 1:
TODO: Test this
```
ros2 run bridge_msgs demo_vrx_write.py
```


