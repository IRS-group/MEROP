# Install ROS:

## Automatic Install:
- `roscd feedback && cd .. && chmod +xrw install.sh && ./install.sh `


## Manual Install:

-  `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' `
-  `sudo apt install curl `
-  `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  `
-  `sudo apt-get update `
-  `sudo apt-get install ros-kinetic-desktop-full `
-  `echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc `
-  `source ~/.bashrc `
-  `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential `
-  `sudo rosdep init `
-  `rosdep update `

-----

- `sudo apt-get insta- ll ros-kinetic-joy `
- `sudo apt-get install ros-kinetic-tf2-geometry-msgs  `
- `sudo apt-get install ros-kinetic-teleop-twist-joy ros-kinetic-rosbridge-suite  `
- `sudo apt-get install ros-kinetic-robot-upstart `
- `sudo apt-get install ffmpeg `
- `sudo apt-get install docker.io `
- `sudo apt install python-pip `
- `pip install transforms3d `
    - #sudo apt-get install v4l-utils 
    - #pip install pymongo
- `sudo apt-get install v4l2loopback-*  `

- pip install opencv-python
 
-----

# CONFIGURATION

## Setup ROS MASTER AND IP

- `export ROS_MASTER_URI=http://<Remote Master IP>:11311/  `
- `export ROS_IP=<Local IP>10.185.1.111  `

THEN need to make the hosts know the ip for the ROS_MASTER

- `sudo gedit /etc/hosts `
     - `<Remote Master IP> <hostname of ros_master>  `


## Setup v4l2loopback

- `sudo echo options v4l2loopback devices=4 video_nr=10,11,12,13 card_label=stereo_left_cam,stereo_right_cam,front_hazard_cam,rear_hazard_cam exclusive_caps=1,1,1,1 max_buffers=2 > /etc/modprobe.d/v4l2loopback.conf`
- `echo v4l2loopback > etc/modules`
- `modprobe v4l2loopback `

## Check Video
- `ffplay -f v4l2 /dev/video10`   - stereo_left_cam
- `ffplay -f v4l2 /dev/video11`   - stereo_right_cam
- `ffplay -f v4l2 /dev/video12`   - front_hazard_cam
- `ffplay -f v4l2 /dev/video13`   - rear_hazard_cam

## Docker
- `sudo docker pull mpromonet/webrtc-streamer `
- `sudo groupadd docker `
- `sudo gpasswd -a $USER docker `
- Do logout and login to finalize the setup


--------------
# Testing Setup:



## MANUAL RUN PACKAGES

- `roslaunch feedback amadee20.launch`
- `roslaunch feedback imagetov4l2loop.launch`
#- `roslaunch map_to_img map_to_img.launch`
- `roslaunch image_2_dev_video image_2_dev_video.launch`
- `roslaunch experiment_record recordBags.launch`


- `docker run --device=/dev/video10 --device=/dev/video11 --device=/dev/video12 --device=/dev/video13 --net=host -p 8000:8000 -it mpromonet/webrtc-streamer -a`


--------------
# STARTUP Nodes 

- `rosrun robot_upstart install feedback/launch/amadee20.launch --job merop-nodes --symlink --master http://<Remote Master IP>:11311 --user merop ` 
- ` rosrun robot_upstart install image_2_dev_video/launch/image_2_dev_video.launch --job merop-v4l2loop --symlink --master http://<Remote Master IP>:11311 --user merop ` 
- `rosrun robot_upstart install experiment_record/launch/recordBags.launch --job merop-recordBags --symlink --master http://<Remote Master IP>:11311 --user merop `
#- `sudo cp ../feedback/scripts/merop-imageConv.service /etc/systemd/system/ `
#- `sudo systemctl daemon-reload`
#- `sudo systemctl enable merop-imageConv.service`
#- `sudo systemctl start merop-imageConv.service `
- `docker run --device=/dev/video10 --device=/dev/video11 --device=/dev/video12 --device=/dev/video13 --net=host -p 8000:8000 -it --restart unless-stopped mpromonet/webrtc-streamer -a`



--------------
# Virtualbox config autostart merop VM (Ubuntu 16.04) - Outside VM environment

- `echo -e "VBOXAUTOSTART_DB=/etc/vbox\nVBOXAUTOSTART_CONFIG=/etc/vbox/autostartvm.cfg" | sudo tee /etc/default/virtualbox `
- `echo -e "default_policy = deny\n\n<US
ER> = {\n   allow = true\n  startup_delay = 10\n}" | sudo tee /etc/vbox/autostartvm.cfg`
- `sudo usermod -aG vboxusers amos`
- `sudo chgrp vboxusers /etc/vbox`
- `sudo chmod g+w /etc/vbox`
- `sudo chmod +t /etc/vbox`
- `VBoxManage setproperty autostartdbpath /etc/vbox/`
- `vboxmanage modifyvm MEROP --autostart-enabled on`
- `sudo systemctl restart vboxautostart-service`
--------------

# STARTUP WAITING FOR ROSMASTER (WIP) 

Does not work, vm will have to be "turned on" manually after roscore and all the rover's cameras are on

Steps taken:
- Added restart field to the services
- Docker is giving problems - image was not appearing on the browser
- Removed all dockers "instances"
- restarted
- docker didn't turn on
- run again the ` docker run --device (...)`
- docker is on, but no video capture device available/running
- restarting com v4l2loop fixed
- everything works only if roscore and rosbag is on
    - roscore only topics works
    - roscore+rosbag image works




