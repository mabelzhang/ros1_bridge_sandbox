# ROS 1 Melodic and ROS 2 Dashing from Debian
# Virtual RobotX (VRX) simulation https://bitbucket.org/osrf/vrx/src/default/

FROM ubuntu:bionic

#####
# ROS 2 Dashing from Debian
# Following instructions to install ROS 2 from Debian
#   https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

# Set up locale
RUN apt update && apt install -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8

# Set up sources
RUN apt update && apt install -y curl gnupg2 lsb-release && \
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# Set up sources.list
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Bypass geographic area prompt
ARG DEBIAN_FRONTEND=noninteractive

# Install Dashing
RUN apt update \
  && apt install -y ros-dashing-desktop

# Install argcomplete for command line tools
RUN apt update \
  && apt install -y python3-argcomplete ros-dashing-rmw-opensplice-cpp


#####
# ROS 1 Melodic from Debian
# Following instructions to install ROS 1 from Debian

# Set up sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install Melodic
RUN apt update \
  && apt install -y ros-melodic-desktop-full

# Dependencies for building packages
RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential


#####
# Create a user, so that can run rosdep update from non-root user
# From subt Docker for gazebo9
#   https://bitbucket.org/osrf/subt/src/gazebo9/docker/subt/Dockerfile

RUN apt update \
 && apt install -y \
    sudo \
    vim \
    libsdl-image1.2-dev \
    libsdl-dev

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME


# Initialize rosdep for ROS 1
RUN sudo rosdep init && rosdep update

USER root

# Install ros1_bridge to talk between ROS 1 and 2
# Requires both ROS 1 and 2 to have been installed
RUN apt update \
  && apt install -y ros-dashing-ros1-bridge


#####
# Optional: VRX robot simulation environment
# From https://bitbucket.org/osrf/vrx/wiki/tutorials

#RUN apt update \
#  && apt install ros-melodic-vrx-gazebo


# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME
USER $USERNAME
