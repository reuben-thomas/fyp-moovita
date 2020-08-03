#!/bin/bash

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# Setup sources list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Kinetic
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
<<<<<<< HEAD
=======
sudo apt-cache search ros-kinetic
>>>>>>> 84fb15e0089cd76c8e316d79800e7f8e40ee2cde

# Setup Environment
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
<<<<<<< HEAD
source /opt/ros/kinetic/setup.bash
=======
>>>>>>> 84fb15e0089cd76c8e316d79800e7f8e40ee2cde

# Install the ROS dependencies for building packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
