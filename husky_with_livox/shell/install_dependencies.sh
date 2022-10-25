# Setup to clone livox repository
sudo apt-get install git
cd ~/catkin_ws/src

# Clones Livox 
git clone git@github.com:Livox-SDK/livox_laser_simulation.git

# Install all packages related to Husky
sudo apt-get install ros-noetic-husky-gazebo
sudo apt-get install ros-noetic-husky-control
sudo apt-get install ros-noetic-husky-description
sudo apt-get install ros-noetic-husky-msgs
sudo apt-get install ros-noetic-husky-simulator
sudo apt-get install ros-noetic-husky-viz
