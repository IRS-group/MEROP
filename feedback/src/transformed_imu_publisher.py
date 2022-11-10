#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, QuaternionStamped
from transforms3d import euler, quaternions as qua

pubCorrectQuat = 0
pubAttitude = 0
quaTrans3D = 0

#def convertTrans3DQautToQuat(q):
#    quat = QuaternionStamped()
#    quat.header.stamp = rospy.Time.now()
#    quat.header.frame_id = "base_link"
#    quat.quaternion.x = q[1]
#    quat.quaternion.y = q[2]
#    quat.quaternion.z = q[3]
#    quat.quaternion.w = q[0]

#    pubCorrectQuat.publish(quat)


def convertQuatToTrans3DQuat(quat):
    global quaTrans3D
    quaTrans3D = (
        quat.w,
        quat.x,
        quat.y,
        quat.z)

def getIMU(imu_msg):
    attmsg = imu_msg.quaternion

    convertQuatToTrans3DQuat(attmsg)

    # Quaternion representing the inverse matrix of [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
    #qua_conj = (0, 0, 1, 0)

    #ori_qua = qua.qmult(qua_conj, quaTrans3D) # Get the original IMU fram of reference

    yaw, roll, pitch = euler.quat2euler(quaTrans3D, 'rzxy')

    yaw = math.degrees(yaw)
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)

    attitude = Vector3(roll,pitch,yaw)

    rospy.loginfo("Attitude > Y: %f, R: %f, P: %f", yaw, roll, pitch)

    #convertTrans3DQautToQuat(ori_qua)

    pubAttitude.publish(attitude)

def attitudeSub():
    global pubAttitude #, pubCorrectQuat
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('attitudeSub', anonymous=True)

    rospy.Subscriber("/navsat/orientation", QuaternionStamped, getIMU)
    pubAttitude = rospy.Publisher('/merop/attitude', Vector3, queue_size=1)
    #pubCorrectQuat = rospy.Publisher('/raposang/transf_imu_quat', QuaternionStamped, queue_size=10)

    #rate = rospy.Rate(10)  # 10hz

    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("IMU transformer is running...")
    rospy.spin()

if __name__ == '__main__':
    try:
        attitudeSub()
    except rospy.ROSInterruptException:
        pass
