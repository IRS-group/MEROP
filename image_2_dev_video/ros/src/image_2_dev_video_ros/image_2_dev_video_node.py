#!/usr/bin/python3
import os
import sys
import cv2
import fcntl
from v4l2 import (
    v4l2_format, VIDIOC_G_FMT, V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_BGR24, V4L2_COLORSPACE_SRGB,
    V4L2_FIELD_NONE, VIDIOC_S_FMT, V4L2_PIX_FMT_YUV420
)

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import numpy as np

VIDEO_OUT = "/dev/video1"

first_image = False
got_camera_info = False
VID_WIDTH = 900
VID_HEIGHT = 200

def callback(msg):
    global image, first_image, output
    # Decompress Image
    np_array = np.fromstring(msg.data, np.uint8)
    image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    frame = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)

    # write to /dev/video
    written = os.write(output, frame.data)
    if written < 0:
        print("ERROR: could not write to output device!")
        os.close(output)

def callback_camera_info(msg):
    global VID_WIDTH, VID_HEIGHT, got_camera_info
    if not got_camera_info:
        VID_WIDTH = msg.width
        VID_HEIGHT = msg.height

        rospy.logwarn(VID_WIDTH)
        rospy.logwarn(VID_HEIGHT)

        got_camera_info = True


def main():
    global output, VIDEO_OUT, VID_WIDTH, VID_HEIGHT
    rospy.init_node('image_to_device', anonymous=False)

    topic_name = rospy.get_param('~image', '/hazard_front/zed_node_front/rgb/image_rect_color/compressed')
    camera_info = rospy.get_param('~camera_info', '/hazard_front/zed_node_front/rgb/camera_info')
    VIDEO_OUT = rospy.get_param('~video_device', '/dev/video10')

    rospy.Subscriber(camera_info, CameraInfo, callback_camera_info, queue_size=1)

    rospy.loginfo("Started converting " + str(topic_name) + " to device " + str(VIDEO_OUT))

    # open output device
    try:
        output = os.open(VIDEO_OUT, os.O_RDWR)
    except Exception as ex:
        print("ERROR: could not open output device!")
        print(str(ex))
        return -1

    # output.set(CV_CAP_PROP_FOURCC, CV_FOURCC ('M', 'J', 'P', 'E', 'G'))

    # Remove this when we have camera_info - rosbag didnt record it
    # VID_WIDTH = 1280
    # VID_HEIGHT = 720
    while not got_camera_info:
        continue

    # configure params for output device
    vid_format = v4l2_format()
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
    if fcntl.ioctl(output, VIDIOC_G_FMT, vid_format) < 0:
        print("ERROR: unable to get video format!")
        return -1

    framesize = VID_WIDTH * VID_HEIGHT * 4
    vid_format.fmt.pix.width = VID_WIDTH
    vid_format.fmt.pix.height = VID_HEIGHT

    # NOTE: change this according to below filters...
    # Chose one from the supported formats on Chrome: YUV420, Y16, Z16, INVZ,
    # YUYV, RGB24, MJPEG, JPEG
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420
    vid_format.fmt.pix.sizeimage = framesize
    vid_format.fmt.pix.field = V4L2_FIELD_NONE
    vid_format.fmt.pix.bytesperline = 0
    vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB

    if fcntl.ioctl(output, VIDIOC_S_FMT, vid_format) < 0:
        print("ERROR: unable to set video format!")
        return -1
    rospy.Subscriber(topic_name, CompressedImage, callback, queue_size=1)
    # rospy.Subscriber('/hazard_front/zed_node_front/rgb/image_rect_color/', Image, callback_raw, queue_size=1)

    # loop over these actions:
    while not rospy.is_shutdown():
        pass


