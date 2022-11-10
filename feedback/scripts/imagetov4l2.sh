#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/merop_ws/devel/setup.bash

rosrun image_to_v4l2loopback stream _device:=/dev/video10 _fourcc:=12 image:=/stereo_camera/left/image_raw &
sleep 2
rosrun image_to_v4l2loopback stream _device:=/dev/video11 _fourcc:=12 image:=/stereo_camera/right/image_raw &


