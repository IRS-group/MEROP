#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/merop_ws/devel/setup.bash

ffmpeg -nostdin -i /dev/video10 -f v4l2 /dev/video20 &
sleep 2
ffmpeg -nostdin -i /dev/video11 -f v4l2 /dev/video21 &
sleep 2
ffmpeg -nostdin -i /dev/video12 -f v4l2 /dev/video22 &
sleep 2
ffmpeg -nostdin -i /dev/video13 -f v4l2 /dev/video23

