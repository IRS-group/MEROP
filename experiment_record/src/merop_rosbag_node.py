#!/usr/bin/env python

import rospy
import shlex
import threading
import subprocess
from experiment_record.msg import RecData
import time
import os

isRecord = False
sp = None
thrd = None

def subpro(*arg):
    global sp
    try:
        sp = subprocess.call(arg, shell=False)
        # if sp < 0:
        #    print("Child was terminated by signal", -retcode)
        # else:
        #    print("Child returned", retcode)
    except OSError as e:
        print("Execution failed:", e)

def recordcall(data):
    global isRecord, sp, thrd

    rospy.logwarn("Inside record callback")

    bagname = "/media/merop/T7/experiment_data/%s_%s_%d" % (data.mission, data.cond, data.subnum)
    shellcom = "rosbag record -o %s /client_count /cmd_vel /cmd_vel_merop /cmd_vel_out /connected_clients /e_stop /elevation_color_map/compressed /elevation_map/compressed /gnss/pose /hazard_front/zed_node_front/rgb/image_rect_color/compressed /hazard_rear/zed_node_rear/rgb/image_rect_color/compressed /joy /joy_teleop/cmd_vel /joy_teleop/joy /map /merop/attitude /cmd_vel_merop /merop/joy /odom /navsat/orientation /rec_bag /slipping_broadcaster /stereo_camera/left/image_raw/compressed /stereo_camera/right/image_raw/compressed /tf /tf_static  __name:=bagrecord" % (
        bagname)
    args = shlex.split(shellcom)
    if data.rec and not isRecord: # to record data has to be true and isrecord false
	rospy.logwarn("Start the recording")        
        thrd = threading.Thread(target = subpro, args= args)
        thrd.start()
        isRecord = data.rec
    elif not data.rec and isRecord: # to stop record data has to be false and isrecord true
	rospy.logwarn("Stop the recording")        
        a = shlex.split("rosnode kill /bagrecord")
        subprocess.call(a, shell=False)
        thrd.join()
        #sp.terminate()
        #sp.communicate()
        #sp = None
        isRecord = data.rec

def rec_rosbag():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('record_rosbag', anonymous=True)

    rospy.Subscriber('/rec_bag', RecData, recordcall)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rec_rosbag()
