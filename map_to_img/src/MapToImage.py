#!/usr/bin/env python

########
# Code adapted from:
#   https://github.com/streklin/home_services_experimental/blob/master/rover_platform/src/nodes/projMapToImage.cpp
#   https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
#   http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
########

import math
import numpy
import rospy
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from transforms3d import euler, quaternions as qua
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from map_to_img.msg import RobotMapPos

rbtTrans_pub = image_pub = mapSize_pub = 0
pose_robotX = pose_robotY = 0
rotation = Quaternion()

def convertQuatToTrans3DQuat(quat):
    qt3D = (
        quat.w,
        quat.x,
        quat.y,
        quat.z)
    return qt3D

def GetRobotPosition(resol, map_OX, map_OY):
    # Robot Position (real world)
    robot_poseX = pose_robotX - map_OX
    robot_poseY = pose_robotY - map_OY
    # Robot position in the map
    final_poseX = robot_poseX/resol
    final_poseY = robot_poseY/resol
    # Publish pose in map
    #rospy.loginfo("X: %d  Y: %d", final_poseX, final_poseY)

    #Robot Orientation
    rot = convertQuatToTrans3DQuat(rotation)
    yaw, pitch, roll  = euler.quat2euler(rot, 'rzyx')
    rospy.loginfo("Yaw: %d  Pitch: %d  Roll: %d", yaw, pitch, roll)

    msg = RobotMapPos()
    msg.translation = Vector3(final_poseX,final_poseY, 0)
    msg.rotation = rotation

    return msg

def ConvertMapImage(map):
    resol = map.info.resolution
    map_width = map.info.width
    map_height = map.info.height

    size_msg = Vector3(map_width, map_height, resol)
    # Map Origin (real world)
    map_OX = map.info.origin.position.x
    map_OY = map.info.origin.position.y

    pos_msg = GetRobotPosition(resol, map_OX, map_OY)

    image = numpy.zeros((map_height,map_width,3), numpy.uint8)

    for x in xrange(map_width):
        for y in xrange(map_height):
            mapPixel = map.data[x+map_width*y]
            #if (abs(x - pos_msg.translation.x) <= 3) and (abs(y - pos_msg.translation.y) <= 3): //this is to check if the position is correct
            #    image[y,x] = [0, 255, 0]
            # elif
            if mapPixel < 0:
                image[y,x] = [150,110,60] # unknown space is equal to colour dark blueish
                #image.itemset((x,y,0),0) #BLUE
                #image.itemset((x,y,1),150) #GREEN
                #image.itemset((x,y,2),255) #RED
            else:
                ratio = 1.0 - mapPixel/100.0
                value = ratio*255
                image[y,x] = [value,value,value]

    flippedImg = cv2.flip(image, 0)  # https://www.pyimagesearch.com/2017/01/02/rotate-images-correctly-with-opencv-and-python/

    bridge = CvBridge()
    cv_msg = bridge.cv2_to_compressed_imgmsg(flippedImg, dst_format = "jpg")


    #use the current time as message time stamp for all messages
    stamp = rospy.Time.now()

    pos_msg.header.stamp = stamp

    ### Alternative use for the cv_bridge code
    #### Create CompressedImage ####
    #msg = numpy.array(cv2.imencode('.jpg', flippedImg)[1]).tostring()
    #imgMsg = CompressedImage()
    #imgMsg.header.stamp = stamp
    #imgMsg.format = "jpeg"
    #imgMsg.data = msg
    #######

    image_pub.publish(cv_msg)
    rbtTrans_pub.publish(pos_msg)
    mapSize_pub.publish(size_msg)


def maptoimage():
    global rbtTrans_pub, mapSize_pub, image_pub, pose_robotX, pose_robotY, rotation
    
    rbtTrans_pub = rospy.Publisher('/map_image/pose', RobotMapPos, queue_size=1)
    mapSize_pub = rospy.Publisher('/map_image/size', Vector3, queue_size=1)

    rospy.init_node('maptoimage', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, ConvertMapImage)
    image_pub = rospy.Publisher('/map_image/compressed', CompressedImage, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            pose_robotX = trans.transform.translation.x
            pose_robotY = trans.transform.translation.y
            rotation = trans.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Could not get tf between map and base_link")
            rate.sleep()
            continue

    rate.sleep()

if __name__ == '__main__':
    try:
        maptoimage()
    except rospy.ROSInterruptException:
        pass
