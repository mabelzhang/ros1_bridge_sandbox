Minimal packages to demonstrate the usage of [`ros1_bridge`](https://github.com/ros2/ros1_bridge), a communication channel between ROS 1 and ROS 2. The bridge allows packages in ROS 1 and ROS 2 to run simultaneously, so that ROS 1 packages can be incrementally migrate to ROS 2.

# Prerequisites

This package has been tested in Ubuntu 18.04 with
* ROS 1 Melodic
* ROS 2 Dashing

A Dockerfile with the prerequisites is provided.


# Basic bridge usage

This example demonstrates basic communication between publisher and subscriber across ROS 1 and ROS 2.

Run the Docker container.
If you do not have Docker installed, you can still follow the instructions below after cloning this repository manually.

Shell 1, compile ROS 1 custom message and run ROS 1 core:
```
. /opt/ros/melodic/setup.bash
cd ros1_bridge_sandbox/ros1_msgs_ws
catkin_make_isolated --install
. devel_isolated/setup.bash
# Run roscore in the background. Press enter to get a clean prompt
roscore &
```

Shell 2, compile ROS 2 custom message and run a publisher:
```
. /opt/ros/dashing/setup.bash
cd ros1_bridge_sandbox/ros2_msgs_ws
colcon build --packages-select bridge_msgs
. install/local_setup.bash
# Publishes bridge_msgs/JointCommand type to /joint_command topic
python3 src/bridge_msgs/src/ros2_pub.py
```

Shell 3, check out and compile `ros1_bridge` from source to recognize the custom message we just compiled above:
```
mkdir -p ros1_bridge_sandbox/bridge_ws/src
cd ros1_bridge_sandbox/bridge_ws/src
git clone https://github.com/ros2/ros1_bridge.git
cd ..
. /opt/ros/melodic/setup.bash
. /opt/ros/dashing/setup.bash
# Source the two new messages we compiled above
. ~/ros1_bridge_sandbox/ros1_msgs_ws/install_isolated/setup.bash 
. ~/ros1_bridge_sandbox/ros2_msgs_ws/install/local_setup.bash 
# Compile ros1_bridge and source it
colcon build --packages-select ros1_bridge --cmake-force-configure
. install/setup.bash
# This should print:   - 'bridge_msgs/JointCommand' (ROS 2) <=> 'bridge_msgs/JointCommand' (ROS 1)
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep bridge
  - 'bridge_msgs/msg/JointCommand' (ROS 2) <=> 'bridge_msgs/JointCommand' (ROS 1)
```

If the `grep` does not print anything, verify that the ROS 1 and ROS 2 messages are both recognized in this shell:
```
$ rosmsg show bridge_msgs/JointCommand
float64 position
$ ros2 msg show bridge_msgs/JointCommand
float64 position
```

and that you sourced the newly compiled bridge:
```
$ . ~/ros1_bridge_sandbox/bridge_ws/install/setup.bash
```

Run the bridge, which will carry messages across ROS 1 and 2:
```
ros2 run ros1_bridge dynamic_bridge
```

Shell 1, test subscribing to ROS 2 messages in ROS 1:
```
rosrun bridge_msgs ros1_sub.py
```
You should see printouts on the screen.

If the executable is not found, make sure you have sourced the package:
```
. devel_isolated/setup.bash 
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


# Bridge in a robot simulation

This example demonstrates communication for a more complex system, a simulated robot. Sample scripts have been written to work with the [VRX simulation environment](https://bitbucket.org/osrf/vrx), which features a maritime surface vehicle. A similar setup should work with any other robot.

## Install VRX simulation world

To try it out on the VRX robot, you will need to install the VRX environment.
Follow the [installation tutorials](https://bitbucket.org/osrf/vrx/wiki/tutorials).
Several installation methods are available.

* [Debian install](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall) is the simplest way:
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

## Demonstration

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

