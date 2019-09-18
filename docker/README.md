Docker image of ROS 1 and ROS 2 installed on the same system, for trying out ``ros1_bridge``.

The VRX environment in the Dockerfile is optional, used for demonstrating the bridge on a simulated robot with GUI.
If you choose to include it, its Gazebo GUI interface will require X to run from Docker.
Consult the [VRX docker installation tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupDocker) and the optional [NVIDIA Docker install](https://bitbucket.org/osrf/vrx/wiki/tutorials/installNvidiaDocker) for more information.

To build the image:
```
$ ./build.bash ros_melodic_dashing_deb_vrx
```
