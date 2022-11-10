#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/merop/merop_ws/devel/setup.bash

rosrun experiment_record merop_rosbag_node.py __name:="record_rosbag"
