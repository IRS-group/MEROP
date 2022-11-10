#!/usr/bin/env python
import rospy
import math
import random

from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16

def angles_minus_pi_to_pi(theta):
    test = -math.pi <= theta < math.pi  # -pi <= theta < pi
    while not test:
        if theta >= math.pi:
            theta = theta - 2 * math.pi
        if theta < 0 - math.pi:
            theta = theta + 2 * math.pi
        test = -math.pi <= theta < math.pi

    return theta


class DataGenerator:
    """
    Objects from this class constantly publishes fake/simulated data
    to test the haptic devices (traction and attitude) from merop team
    during amadee-20 experiments
    """
    def __init__(self):
        rospy.init_node('fake_data_publisher', anonymous=False)
        self.pub_traction = rospy.Publisher('fake_traction', Int16, queue_size=1)
        self.pub_pose_euler = rospy.Publisher('fake_pose', Vector3, queue_size=1)

        # rate object to setup the frequency of the node:
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 5.0))
        # frequency (float) of the node
        self.frequency = rospy.get_param('~loop_rate', 5.0)
        # time duration of a full turn on the attitude device -> one turn will generate a full conic movement
        self.full_rotation_duration = rospy.get_param('~full_rotation_duration', 3.0)

        self.vibration_duration_param = rospy.get_param('~vib_duration', 2.0)
        self.traction_duration_last_change = rospy.get_time()

    def start_data_generator(self):
        # initialize theta (angle to generate attitude data)
        theta = -math.pi
        # initialize traction state variable
        traction_state = -1

        rospy.loginfo("Publishing fake data (traction and attitude) to test haptic devices...")
        while not rospy.is_shutdown():
            theta = self.publish_attitude(theta)
            traction_state = self.publish_traction(traction_state)
            self.loop_rate.sleep()

    def angle_sample_calculation(self, d_previous):
        """
        D_k = D_{k-1} + (2*pi)/(f*d_{full_rotation})
        :return: D_k
        """
        d = d_previous + (2 * math.pi) / (self.frequency * self.full_rotation_duration)
        # ensure that:  -pi < d =< pi
        d = angles_minus_pi_to_pi(d)
        return d

    def publish_attitude(self, theta):
        """
        publishes a new imu data regarding simulated attitude data
        :param theta:
        :return: theta
        """
        # calculate current angle sample to be be published
        theta = self.angle_sample_calculation(theta)
        # get roll and pitch values (degrees) to generate conic movement
        w_x = math.sin(theta) * 180 / math.pi  # roll
        w_y = math.cos(theta) * 180 / math.pi  # pitch
        # publish attitude data
        self.pub_pose_euler.publish(Vector3(w_x, w_y, 0.0))

        return theta

    def publish_traction(self, traction_state):
        """
        Traction States meaning:
            1 = stuck
            2 = sliding
            -1 = nominal
        :return:
        """
        # all possible traction states:
        traction_states = [-1, 1, 2]

        # change the traction state after a certain time interval (self.vibration_duration_param)
        if (rospy.get_time() - self.traction_duration_last_change) > self.vibration_duration_param:
            previous = traction_state
            # ensure that the random pic is always different from the last one
            while previous == traction_state:
                traction_state = random.choice(traction_states)
            # reset the timer for the traction duration
            self.traction_duration_last_change = rospy.get_time()

        # publish traction data/state
        self.pub_traction.publish(Int16(traction_state))

        return traction_state

def main():

    mock_up_data = DataGenerator()
    mock_up_data.start_data_generator()

if __name__ == '__main__':
    main()

