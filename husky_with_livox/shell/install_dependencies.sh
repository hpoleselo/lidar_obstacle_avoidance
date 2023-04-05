# Makes sure to source ROS, otherwise $ROS_DISTRO won't work below
source /opt/ros/noetic/setup.bash

# Setup to clone livox repository
sudo apt-get install git
cd ~/catkin_ws/src

# Clones Livox 
git clone git@github.com:Livox-SDK/livox_laser_simulation.git

# Clones LaR environment
git clone https://github.com/lar-deeufba/lar_gazebo.git

# Install Ignition dependencies
sudo apt-get install libignition-math4*

# Install all packages related to Husky
sudo apt-get install ros-${ROS_DISTRO}-husky-gazebo
sudo apt-get install ros-${ROS_DISTRO}-husky-control
sudo apt-get install ros-${ROS_DISTRO}-husky-description
sudo apt-get install ros-${ROS_DISTRO}-husky-msgs
sudo apt-get install ros-${ROS_DISTRO}-husky-simulator
sudo apt-get install ros-${ROS_DISTRO}-husky-viz

cd ~/catkin_ws
catkin_make
