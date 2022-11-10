#!/usr/bin/python3

import os
import sys
import cv2
import fcntl
from v4l2 import (
    v4l2_format, VIDIOC_G_FMT, V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_BGR24,
    V4L2_FIELD_NONE, VIDIOC_S_FMT
)

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import numpy as np

VIDEO_IN = "/dev/video0"
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
    # write to /dev/video
    written = os.write(output, image.data)
    if written < 0:
        print("ERROR: could not write to output device!")
        os.close(output)
    # rospy.loginfo("got image")

def callback_camera_info(msg):
    global VID_WIDTH, VID_HEIGHT, got_camera_info
    if not got_camera_info:
        VID_WIDTH = msg.width
        VID_HEIGHT = msg.height

        rospy.logwarn(VID_WIDTH)
        rospy.logwarn(VID_HEIGHT)

        got_camera_info = True


def main():
    global output
    rospy.init_node('image_to_device', anonymous=False)
    rospy.Subscriber('/hazard_front/zed_node_front/rgb/camera_info', CameraInfo, callback_camera_info,
                     queue_size=1)

    rospy.loginfo("Started converting  /hazard_front/zed_node_front/rgb/camera_info  to device  /dev/video1")

    # # open and configure input camera
    # cam = cv2.VideoCapture(VIDEO_IN)
    # if not cam.isOpened():
    #     print("ERROR: could not open camera!")
    #     return -1

    # cam.set(cv2.CAP_PROP_FRAME_WIDTH, VID_WIDTH)
    # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, VID_HEIGHT)

    # open output device
    try:
        output = os.open(VIDEO_OUT, os.O_RDWR)
    except Exception as ex:
        print("ERROR: could not open output device!")
        print(str(ex))
        return -1

    while not got_camera_info:
        continue

    # configure params for output device
    vid_format = v4l2_format()
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
    if fcntl.ioctl(output, VIDIOC_G_FMT, vid_format) < 0:
        print("ERROR: unable to get video format!")
        return -1

    framesize = VID_WIDTH * VID_HEIGHT * 3
    vid_format.fmt.pix.width = VID_WIDTH
    vid_format.fmt.pix.height = VID_HEIGHT

    # NOTE: change this according to below filters...
    # Chose one from the supported formats on Chrome: YUV420, Y16, Z16, INVZ,
    # YUYV, RGB24, MJPEG, JPEG
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24
    vid_format.fmt.pix.sizeimage = framesize
    vid_format.fmt.pix.field = V4L2_FIELD_NONE

    if fcntl.ioctl(output, VIDIOC_S_FMT, vid_format) < 0:
        print("ERROR: unable to set video format!")
        return -1
    # rospy.sleep(2)
    rospy.Subscriber('/hazard_front/zed_node_front/rgb/image_rect_color/compressed', CompressedImage, callback,
                     queue_size=1)

    # create GUI window
    # gui = "gui"
    # cv2.namedWindow(gui)
    # cv2.setWindowTitle(gui, "OpenCV test")

    # loop over these actions:
    while not rospy.is_shutdown():
        try:
            # frame = image
            # apply simple filter (NOTE: result should be as defined PIXEL FORMAT)
            # convert twice because we need RGB24
            # result = cv2.cvtColor(frame, cv2.CV_8UC3)
            # result = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # show frame
            # cv2.imshow(gui, image)

            # write frame to output device
            # written = os.write(output, image.data)
            # if written < 0:
            #     print("ERROR: could not write to output device!")
            #     os.close(output)
            #     break
            pass
            # wait for user to finish program pressing ESC
            # if cv2.waitKey(10) == 27:
            #     rospy.signal_shutdown()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    main()
