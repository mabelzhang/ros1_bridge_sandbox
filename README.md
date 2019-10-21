Minimal packages to demonstrate the usage of [`ros1_bridge`](https://github.com/ros2/ros1_bridge), a communication channel between ROS 1 and ROS 2. The bridge allows packages in ROS 1 and ROS 2 to run simultaneously, so that ROS 1 packages can be incrementally migrate to ROS 2.

## Prerequisites

This package has been tested in Ubuntu 18.04 with
* ROS 1 Melodic
* ROS 2 Dashing

A Dockerfile with the prerequisites is provided.


## Basic bridge usage

This example demonstrates basic communication between publisher and subscriber across ROS 1 and ROS 2.

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


## Bridge in a robot simulation

This example demonstrates communication for a more complex system, a simulated robot. Sample scripts have been written to work with the [VRX simulation environment](https://bitbucket.org/osrf/vrx), which features a maritime surface vehicle. A similar setup should work with any other robot.

### Install VRX simulation world

To try it out on the VRX robot, you will need to install the VRX environment.
Follow the [installation tutorials](https://bitbucket.org/osrf/vrx/wiki/tutorials).
There are several ways to install.

* The simplest installation method is the [Debian install](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall):
   ```
   sudo apt install ros-melodic-vrx-gazebo
   ```

* Alternatively, you can [install from source](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).
Clone the VRX simulation repository to a desired location, compile and source the packages:
   ```
   mkdir -p vrx_ws/src
   cd vrx_ws/src
   hg clone https://bitbucket.org/osrf/vrx
   . /opt/ros/melodic/setup.bash
   cd ..
   catkin build
   . devel/setup.bash
   ```

* A third alternative is to [install the VRX Docker image](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupDocker). Docker X-server support is required to run Gazebo in Docker. If you choose this option, you may want to [install NVIDIA Docker](https://bitbucket.org/osrf/vrx/wiki/tutorials/installNvidiaDocker) to speed up rendering.

### Demonstration

Once the VRX environment is installed, run the following commands in several new terminals.

Shell 1, launch VRX simulation in ROS 1:
```
roslaunch vrx_gazebo vrx.launch
```

Shell 2, run RViz 2 in ROS 2:
```
. /opt/ros/dashing/setup.bash
ros2 run rviz2 rviz2
```
Add Image topics and LIDAR topic to RViz, or use the sample ``vrx_config.rviz`` RViz configuration file.

Shell 3, run the bridge:
```
. /opt/ros/melodic/setup.bash
. /opt/ros/dashing/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

Shell 4, run ROS 2 node to subscribe to built-in data types in ROS 1:
TODO: Test this
```
ros2 run bridge_msgs demo_vrx_read.py
```

Run ROS 2 node to publish to built-in data types in ROS 1:
TODO: Test this
```
ros2 run bridge_msgs demo_vrx_write.py
```

