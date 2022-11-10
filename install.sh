#!/bin/sh
# setup for merop's VM necessary packages

# install ROS:

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  
sudo apt-get update 
sudo apt-get install -y ros-kinetic-desktop-full 
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc 
source ~/.bashrc 
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential 
sudo rosdep init 
rosdep update 

# create ROS Worspace
cd 
mkdir ros_ws/src

# Install ROS PACKAGES
sudo apt-get install -y ros-kinetic-joy 
sudo apt-get install -y ros-kinetic-tf2-geometry-msgs
sudo apt-get install -y ros-kinetic-teleop-twist-joy ros-kinetic-rosbridge-suite
sudo apt-get install -y ffmpeg
sudo apt-get install -y docker.io
sudo apt install -y python-pip
pip install -y transforms3d
sudo apt-get install v4l2loopback-*

# Install and setup Docker
sudo docker pull mpromonet/webrtc-streamer
sudo groupadd docker
sudo gpasswd -a $USER docker

# Install other useful items 
sudo apt install tree
sudo apt-get install terminator
sudo update-alternatives --config x-terminal-emulator
sudo apt-get install ssh

