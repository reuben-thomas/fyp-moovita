#!/bin/bash

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# apt-get installs
sudo apt-get update
sudo apt-get install gazebo7
sudo apt-get install libgazebo7-dev
sudo apt-get install git
sudo apt-get install ros-kinetic-fake-localization
sudo apt-get install joint-state-publisher
sudo apt-get install ros-kinetic-joint-state-publisher-gui
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

# git clones
git clone https://github.com/ros-drivers/ackermann_msgs.git

# pip installs
pip install numpy
