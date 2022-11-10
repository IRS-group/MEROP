#!/usr/bin/env python
import math
import sys
import matplotlib.pyplot as plt
import tf
import rospy
import numpy as np

from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class trajectory_data:
    def __init__(self):
        self.x = []
        self.y = []

        self.theta = []
        self.time = []

    def add_new_position(self, x, y, theta, time):
        self.x.append(x)
        self.y.append(y)
        self.theta.append(theta)
        self.time.append(time)
        if self.time[-1] - self.time[0] > 150:  # only keep a buffer of 2.5 minutes = 150 seconds; this only affects the graph
            self.x.pop()
            self.y.pop()
            self.theta.pop()
            self.time.pop()


class position_data:
    def __init__(self):
        self.x = []
        self.y = []
        self.time = []

    def add_value(self, x, y, time):
        self.x.append(x)
        self.y.append(y)
        self.time.append(time)
        if self.time[-1] - self.time[0] > 150:  # only keep a buffer of 2.5 minutes = 150 seconds; this only affects the graph
            self.x.pop()
            self.y.pop()
            self.time.pop()


class rotation_data:
    def __init__(self):
        self.theta = []
        self.time = []

    def add_value(self, theta, time):
        self.theta.append(theta)
        self.time.append(time)
        if self.time[-1] - self.time[0] > 150:  # only keep a buffer of 2.5 minutes = 150 seconds; this only affects the graph
            self.theta.pop()
            self.time.pop()

    def latest(self):
        if len(self.theta) > 0:
            return self.theta[-1]
        else:
            return 0


class distance_data:
    def __init__(self):
        self.dist = []
        self.time = []

    def add_value(self, theta, time):
        self.dist.append(theta)
        self.time.append(time)
        if self.time[-1] - self.time[0] > 150:  # only keep a buffer of 2.5 minutes = 150 seconds; this only affects the graph
            self.dist.pop()
            self.time.pop()

    def latest(self):
        if len(self.dist) > 0:
            return self.dist[-1]
        else:
            return 0


class difference_data:
    def __init__(self):
        self.dif = []
        self.time = []

    def add_value(self, dif, time):
        self.dif.append(dif)

        self.time.append(time)
        if self.time[-1] - self.time[0] > 150:  # only keep a buffer of 2.5 minutes = 150 seconds; this only affects the graph
            self.dif.pop()
            self.time.pop()

    def latest(self):
        if len(self.dif) > 0:
            return self.dif[-1]
        else:
            return 0


class measurements:
    def __init__(self):
        self.position = position_data()
        self.distance = distance_data()
        self.rotation = rotation_data()
        self.trajectory = trajectory_data()

    def calculate_distance(self):
        dist = math.sqrt(self.position.x[-1] * self.position.x[-1] + self.position.y[-1] * self.position.y[-1])
        return dist


# ----------------------------------------------------------------------------------------- #
#                           MAIN OBJECT - TRACTION DETECTOR                                 #
# ----------------------------------------------------------------------------------------- #
class traction_detector:
    def __init__(self):
        self.visual_odom_d = measurements()
        self.odometry_d = measurements()
        self.position_difference = difference_data()
        self.orientation_difference = difference_data()

        self.current_time = 0  # variable is currently only used for graph representation and debug
        self.update_visual_odom = True
        self.update_odom = True
        self.got_first_odom_measurement = False
        self.got_first_visual_measurement = False

        # -----------------------DEBUGING/GRAPHS----------------------------- #
        self.threshold_position = distance_data()
        self.threshold_orientation = distance_data()
        self.debug_mode = rospy.get_param('/traction_detection/debug_mode', True)

        # ----------------------DETECTION PARAMETERS------------------------- #
        # VALUES USED DURING THE USER_STUDY:
        self.minimum_threshold_linear = rospy.get_param('/traction_detection/minimum_threshold_linear', 0.07)
        self.minimum_threshold_angular = rospy.get_param('/traction_detection/minimum_threshold_angular', 0.266)
        self.thresh_factor = rospy.get_param('/traction_detection/threshold_factor', 0.7)
        self.frequency = rospy.get_param('/traction_detection/frequency', 10.0)

        # ----------------------FILTERING------------------------- #
        self.T_sample = 1.0 / self.frequency
        # self.T_sample = 0.1
        # self.size_moving_window = 7
        time_window = rospy.get_param('/traction_detection/time_moving_window', 0.7)  # seconds
        self.size_moving_window = int(time_window * self.frequency)  # n

        # ----------------------NODE SETUP------------------------- #
        rospy.init_node('traction_detection')
        if self.debug_mode:
            rospy.on_shutdown(self.data_representation)
        self.pub = rospy.Publisher('slipping_broadcaster', Int16, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
        # rospy.Subscriber("/odom_visual_pose", PoseStamped, self.callback_visual_odom)
        # rospy.Subscriber("/hazard_front/zed_node_front/odom", Odometry, self.callback_odom_visual)
        rospy.Subscriber("/gnss/pose", PoseWithCovarianceStamped, self.callback_gnss_pose)
        # rospy.Subscriber("/pose2D", Pose2D, self.callback_posed2D)  # laser_scan_matcher: data from laser/point cloud
        # TODO: ask for a rosparam and subscribe to only one topic instead of all

        # SAMPLING FREQUENCY:
        self.rate = rospy.Rate(self.frequency)  # hz

        # ----------------------NODE SETUP------------------------- #
        # only stops indicating loss of traction if 3 nominal states are detected
        self. past_state = -1
        self.nominal_count = 0

    # ----------------------------------------------------------------------------------------- #
    #                       MAIN FUNCTION - TRACTION DETECTION                                  #
    # ----------------------------------------------------------------------------------------- #
    def start_detection(self):
        rospy.loginfo("starting traction detector...")
        rospy.logwarn_throttle(100000000, "Waiting for both odometry sources to start traction detection")

        while not rospy.is_shutdown():
            try:
                if self.got_first_odom_measurement and self.got_first_visual_measurement:
                    rospy.logwarn_throttle(100000000, "Got both odometry sources... will start traction detection")
                    self.estimate_traction()
                    self.current_time = self.current_time + self.T_sample
            except rospy.ROSInterruptException:
                pass

            self.rate.sleep()

            # trigger flag to obtain new odometry data
            self.update_visual_odom = True
            self.update_odom = True

    # ----------------------------------------------------------------------------------------- #
    #                               TRACTION DETECTION                                          #
    # ----------------------------------------------------------------------------------------- #
    def estimate_traction(self):

        # calculate displacements (odom vs visual) to obtain traction
        d_p_odom, d_p_visual_odom, d_odom, d_visual = self.all_displacement_calculation()

        # compare displacements and obtains differences
        self.compare_displacements(d_p_odom, d_p_visual_odom, d_odom, d_visual)

        # check if any displacement comparison was done that needs publishing:
        if len(self.position_difference.dif) > 0:
            self.publish_slippage_position(d_p_odom, d_p_visual_odom)

    # ----------------------------------------------------------------------------------------- #
    #                              DISPLACEMENT CALCULATION                                     #
    # ----------------------------------------------------------------------------------------- #
    def all_displacement_calculation(self):
        """
        Based on the latest and previous odometry measures obtain the displacement during a certain time interval (1/f)
        :return: wheel_odom_displacement, visual_odom_displacement, [d_x_odom, d_y_odom, d_theta_odom],
                                                                    [d_x_visual, d_y_visual, d_theta_visual]
        """
        d_x_odom, d_y_odom, d_theta_odom = self.single_displacement_estimation(self.odometry_d.trajectory.x,
                                                                               self.odometry_d.trajectory.y,
                                                                               self.odometry_d.trajectory.theta)
        # Displacement Vector from WHEEL ODOMETRY
        d_p_odom = np.array([d_x_odom, d_y_odom])

        d_x_visual_odom, d_y_visual_odom, d_theta_visual_odom = self.single_displacement_estimation(
            self.visual_odom_d.trajectory.x, self.visual_odom_d.trajectory.y, self.visual_odom_d.trajectory.theta)

        # Displacement Vector from VISUAL ODOMETRY
        d_p_visual_odom = np.array([d_x_visual_odom, d_y_visual_odom])

        return d_p_odom, d_p_visual_odom, [d_x_odom, d_y_odom, d_theta_odom], [d_x_visual_odom, d_y_visual_odom,
                                                                               d_theta_visual_odom]

    # ----------------------------------------------------------------------------------------- #
    #                              DISPLACEMENT CALCULATION                                     #
    # ----------------------------------------------------------------------------------------- #
    def single_displacement_estimation(self, list_x, list_y, list_theta):
        """
        Generic mathematical displacement calculator
        :param list_x:
        :param list_y:
        :param list_theta:
        :return: d_x = displacement in x axis, d_y = displacement in y axis, d_theta = displacement around z axis
        """
        t0 = -self.size_moving_window  # previous
        t1 = -1  # current

        if len(list_theta) > -t0:

            #   d_x_1 = cos(theta_0) * (x_1 -x_0) + sin(theta_0) * (y_1 -y_0)
            d_x = math.cos(list_theta[t0]) * (list_x[t1] - list_x[t0]) + math.sin(list_theta[t0]) * (
                    list_y[t1] - list_y[t0])

            #   d_y_1 = -sin(theta_0) * (x_1 -x_0) + cos(theta_0) * (y_1 -y_0)
            d_y = -math.sin(list_theta[t0]) * (list_x[t1] - list_x[t0]) + math.cos(list_theta[t0]) * (
                    list_y[t1] - list_y[t0])

            theta0 = self.angles_minus_pi_to_pi(list_theta[t0])
            theta1 = self.angles_minus_pi_to_pi(list_theta[t1])
            d_theta = self.angles_minus_pi_to_pi(theta1 - theta0)

        else:  # coping with first seconds without measurements
            d_x = 0
            d_y = 0
            d_theta = 0
        return d_x, d_y, d_theta

    # ----------------------------------------------------------------------------------------- #
    #                              DISPLACEMENT COMPARISON                                      #
    # ----------------------------------------------------------------------------------------- #
    def compare_displacements(self, d_p_odom, d_p_visual_odom, d_odom, d_visual):
        """
        Compare displacements and obtain its differences (position and orientation)
        :param d_p_odom: displacement vector (only position) given by wheel odometry (includes rotation info)
        :param d_p_visual_odom: displacement vector (only position) given by visual odometry
        :param d_odom: displacement vector given by wheel odometry (includes rotation info)
        :param d_visual: displacement vector given by visual odometry (includes rotation info)
        :return: dif_vector: difference (vector) between real (visual) and expected (wheel) displacements
        """
        # Eliminate crazy big/ non-sense values
        if d_odom[0] < 95 and d_visual[0] < 95 and d_odom[1] < 95 and d_visual[1] < 95:
            # GET DISPLACEMENT DIFFERENCES AND RESPECTIVE MODULE:
            dif_vector = d_p_odom - d_p_visual_odom
            dif_module = self.vector_size(dif_vector)
            self.position_difference.add_value(dif_module, self.current_time)

            # MODULE OF DISPLACEMENT VECTORS OF POSITION:
            displacement_odom = self.vector_size(d_p_odom)
            displacement_visual = self.vector_size(d_p_visual_odom)

            # SAVING DATA FOR ANALYSIS - Traction Queries
            self.odometry_d.distance.add_value(displacement_odom, self.current_time)
            self.visual_odom_d.distance.add_value(displacement_visual, self.current_time)

        # Eliminate crazy big/ non-sense values
        if abs(d_odom[2]) < 1500 and abs(d_visual[2]) < 1500:
            self.odometry_d.rotation.add_value(d_odom[2], self.current_time)
            self.visual_odom_d.rotation.add_value(d_visual[2], self.current_time)

            dif = abs(self.visual_odom_d.rotation.latest() - self.odometry_d.rotation.latest())
            self.orientation_difference.add_value(dif, self.current_time)

    # ----------------------------------------------------------------------------------------- #
    #           TRACTION STATE BUFFER - Only stops sending losses of traction when its sure     #
    # ----------------------------------------------------------------------------------------- #
    # function added to AMADEE-20 - necessary because of gnss and necessary conservative parameters (ymal)
    # chosen parameters avoid false positives (to not create lost of trust on the information)
    def traction_buffer(self, current_state):
        if current_state == -1:
            self.nominal_count += 1
        else:
            self.nominal_count = 0

        # Maintain traction loss data until we are sure it regained traction
        if current_state == -1 and 2 >= self.nominal_count:
            detected_state = self.past_state
        elif current_state != -1:
            detected_state = current_state
        else:
            detected_state = current_state
        self.past_state = detected_state

        return detected_state


    # ----------------------------------------------------------------------------------------- #
    #                    EXPECTED MOVEMENT CALLBACK (WHEEL ODOMETRY)                            #
    # ----------------------------------------------------------------------------------------- #
    def publish_slippage_position(self, d_p_odom, d_p_visual_odom):
        """
        :param d_p_odom:
        :param d_p_visual_odom:
        :return:
        """

        threshold_linear_current = self.update_threshold(self.odometry_d.distance.dist,
                                                         self.visual_odom_d.distance.dist,
                                                         self.minimum_threshold_linear,
                                                         self.threshold_position)
        threshold_angular_current = self.update_threshold(self.odometry_d.rotation.theta,
                                                          self.visual_odom_d.rotation.theta,
                                                          self.minimum_threshold_angular,
                                                          self.threshold_orientation)

        if len(self.position_difference.dif) > 0:
            slipping = self.calculate_traction_state(threshold_linear_current, threshold_angular_current, d_p_odom,
                                                     d_p_visual_odom)
            # ensure that we are traction losses is completly over
            state_buffered = self.traction_buffer(slipping)
            self.pub.publish(state_buffered)

    # ----------------------------------------------------------------------------------------- #
    #                           CALCULATE CURRENT THRESHOLD VALUES                              #
    # ----------------------------------------------------------------------------------------- #
    def update_threshold(self, list_vel_odom, list_vel_laser, minimum_threshold, threshold_list):
        """
        Obtain current threshold (this one is variable and dependent on the displacement magnitude)
        :param list_vel_odom:
        :param list_vel_laser:
        :param minimum_threshold:
        :param threshold_list:
        :return:
        """
        if len(list_vel_odom) > 0 and len(list_vel_laser) > 0:
            thresh = max(abs(list_vel_odom[-1]), abs(list_vel_laser[-1])) * self.thresh_factor
            if thresh < minimum_threshold:
                thresh = minimum_threshold
            threshold_list.add_value(thresh, self.current_time)
            return thresh

    # ----------------------------------------------------------------------------------------- #
    #                               GET TRACTION STATE                                          #
    # ----------------------------------------------------------------------------------------- #
    def calculate_traction_state(self, threshold_linear_current, threshold_angular_current, d_p_odom, d_p_visual_odom):
        """
        Calculate the traction state based on a set of comparisons between current displacements and thresholds
        :param threshold_linear_current:
        :param threshold_angular_current:
        :param d_p_odom:
        :param d_p_visual_odom:
        :return:
        """
        test1, test2, test3, test4 = self.query_slippage(threshold_linear_current, threshold_angular_current)
        slip_int = self.test_slip_and_get_int(test1, test2, test3, test4, d_p_odom, d_p_visual_odom)

        return slip_int

    # ----------------------------------------------------------------------------------------- #
    #                         QUERIES TO GET TRACTION STATE                                     #
    # ----------------------------------------------------------------------------------------- #
    def query_slippage(self, threshold_linear_current, threshold_angular_current):
        # DID IT LOST TRACTION IN POSITION?
        test1 = self.position_difference.dif[-1] > threshold_linear_current

        # DID IT LOST TRACTION IN ORIENTATION ?
        if len(self.orientation_difference.dif) > 0:
            test2 = self.orientation_difference.dif[-1] > threshold_angular_current
        else:
            test2 = False

        # ARE THE WHEELS MOVING MORE THAN THE VISUAL ODOMETRY?
        test3 = abs(self.odometry_d.distance.dist[-1]) > abs(self.visual_odom_d.distance.dist[-1])  # POSITION

        if len(self.odometry_d.rotation.theta) > 0:  # ORIENTATION
            test4 = abs(self.odometry_d.rotation.theta[-1]) > abs(self.visual_odom_d.rotation.theta[-1])
        else:
            test4 = False

        return test1, test2, test3, test4

    # ----------------------------------------------------------------------------------------- #
    #                         QUERIES TO GET TRACTION STATE                                     #
    # ----------------------------------------------------------------------------------------- #
    def test_slip_and_get_int(self, test1, test2, test3, test4, d_p_odom, d_p_visual):
        # did it lose traction?
        #   test1   -> position (True = traction loss)
        #   test2   -> orientation/rotation
        # Wheels moving more than the real robot?
        #   test3: |dif_pos_odom| > |dif_pos_laser|   -> position
        #   test4: |dif_rot_odom| > |dif_rot_laser|   -> orientation/rotation

        if not test1 and not test2:
            return -1

        if test1:
            if test3:
                return 1
            else:
                return 2  # Sliding (linear traction): wheels stopped robot moving

        if test2:
            if test4:
                return 3  # Stuck (angular traction): wheels moving robot stopped
            else:
                return 4  # Sliding (angular traction): wheels stopped robot moving

        return -1



    # ----------------------------------------------------------------------------------------- #
    #              VERIFY SMALL MAGNITUDE DATA AND VECTOR CALCULATIONS                          #
    # ----------------------------------------------------------------------------------------- #
    def testing_small_linear_movement(self, data_vector):
        size = math.sqrt(data_vector[0] * data_vector[0] + data_vector[1] * data_vector[1])
        testing = size < 0.03
        return testing

    def testing_small_angular_movement(self, odom_vector, visual_vector):
        alpha = self.angle_between(odom_vector, visual_vector) * 180 / math.pi
        # assuming that the amplitude of the noise is contained in 1 degree
        test1 = alpha >= 0
        test2 = alpha < 1
        testing = test1 and test2
        return testing

    def angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::3    """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def vector_size(self, v1):
        size = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
        return size

    # ----------------------------------------------------------------------------------------- #
    #                    TRUE MOVEMENT CALLBACK (VISUAL ODOMETRY)                               #
    # ----------------------------------------------------------------------------------------- #
    def callback_visual_odom(self, data):
        if not self.got_first_visual_measurement:
            rospy.loginfo("Got first Visual Odometry measurement ...")
            self.got_first_visual_measurement = True

        if self.update_visual_odom:
            # convert quaternion from pose into euler angles
            quaternion = (
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # traction losses regarding rotation only consider rotation in z (yaw)
            yaw = euler[2]
            # save latest data for comparison and detection
            self.visual_odom_d.trajectory.add_new_position(data.pose.position.x,
                                                           data.pose.position.y,
                                                           yaw,
                                                           self.current_time)
            # only saves data that occur at certain frequency (defined in main)
            self.update_visual_odom = False

    def callback_gnss_pose(self, data):
        if not self.got_first_visual_measurement:
            rospy.loginfo("Got first GNSS Pose measurement ...")
            self.got_first_visual_measurement = True

        if self.update_visual_odom:
            # convert quaternion from pose into euler angles
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # traction losses regarding rotation only consider rotation in z (yaw)
            yaw = euler[2]
            # save latest data for comparison and detection
            self.visual_odom_d.trajectory.add_new_position(data.pose.pose.position.x,
                                                           data.pose.pose.position.y,
                                                           yaw,
                                                           self.current_time)
            # only saves data that occur at certain frequency (defined in main)
            self.update_visual_odom = False

    # ----------------------------------------------------------------------------------------- #
    #                    TRUE MOVEMENT CALLBACK (VISUAL ODOMETRY)                               #
    # ----------------------------------------------------------------------------------------- #
    def callback_odom_visual(self, data):
        if not self.got_first_visual_measurement:
            rospy.loginfo("Got first Visual Odometry measurement ...")
            self.got_first_visual_measurement = True

        if self.update_visual_odom:
            quaternions = (data.pose.pose.orientation.x,
                           data.pose.pose.orientation.y,
                           data.pose.pose.orientation.z,
                           data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternions)
            self.visual_odom_d.trajectory.add_new_position(data.pose.pose.position.x,
                                                           data.pose.pose.position.y,
                                                           euler[-1],
                                                           self.current_time)
            self.update_visual_odom = False

    # ----------------------------------------------------------------------------------------- #
    #                    TRUE MOVEMENT CALLBACK (VISUAL ODOMETRY)                               #
    # ----------------------------------------------------------------------------------------- #
    def callback_posed2D(self, data):
        if not self.got_first_visual_measurement:
            rospy.loginfo("Got first Laser Odometry measurement ...")
            self.got_first_visual_measurement = True

        if self.update_visual_odom:
            self.visual_odom_d.trajectory.add_new_position(data.x, data.y, data.theta, self.current_time)
            self.update_visual_odom = False

    # ----------------------------------------------------------------------------------------- #
    #                    EXPECTED MOVEMENT CALLBACK (WHEEL ODOMETRY)                            #
    # ----------------------------------------------------------------------------------------- #
    def callback_odom(self, data):
        if not self.got_first_odom_measurement:
            rospy.loginfo("Got first Wheel Odometry measurement ...")
            self.got_first_odom_measurement = True

        if self.update_odom:
            quaternions = (data.pose.pose.orientation.x,
                           data.pose.pose.orientation.y,
                           data.pose.pose.orientation.z,
                           data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternions)
            self.odometry_d.trajectory.add_new_position(data.pose.pose.position.x,
                                                        data.pose.pose.position.y,
                                                        euler[-1],
                                                        self.current_time)
            self.update_odom = False

    # ------------------------------------------------------------------------------------------ #
    #                SIMPLE MOVING AVERAGE FILTERING - ANY MEASUREMENT                           #
    # ------------------------------------------------------------------------------------------ #
    def filter_sma(self, new_measurement, sma, p_m_values):
        p_new = new_measurement
        p_old = p_m_values.pop(0)
        p_m_values.append(p_new)
        sma = sma + p_new / self.size_moving_window - p_old / self.size_moving_window
        return sma, p_m_values

    # ------------------------------------------------------------------------------------------ #
    #                       MAINTAIN ANGLES BETWEEN -PI TO PI                                    #
    # ------------------------------------------------------------------------------------------ #
    def angles_minus_pi_to_pi(self, theta):
        # test = theta >= -math.pi and theta < math.pi  # -pi < theta < pi
        test = -math.pi <= theta < math.pi  # -pi < theta < pi
        while not test:
            if theta >= math.pi:
                theta = theta - 2 * math.pi
            if theta < 0 - math.pi:
                theta = theta + 2 * math.pi
            # test = theta >= -math.pi and theta < math.pi
            test = -math.pi <= theta < math.pi
        return theta

    # ------------------------------------------------------------------------------------------ #
    #               Graphs for data visualization and parameter adjustment                       #
    # ------------------------------------------------------------------------------------------ #
    def data_representation(self):

        odometry_d = self.odometry_d
        laser_d = self.visual_odom_d
        threshold_position = self.threshold_position
        position_difference = self.position_difference
        threshold_orientation = self.threshold_orientation
        orientation_difference = self.orientation_difference

        params = {'legend.fontsize': 10,
                  'axes.labelsize': 15,
                  'axes.titlesize': 13,
                  'xtick.labelsize': 10,
                  'ytick.labelsize': 10}
        plt.rcParams.update(params)

        # GRAPHS:
        plt.figure(1, figsize=(12, 6))

        plt.subplot(2, 2, 1)
        plt.xlabel(r'$t_k$')
        plt.ylabel(r'$\Delta p(t_k)$')
        plt.title('Displacement: position')
        plt.plot(odometry_d.distance.time, odometry_d.distance.dist, 'b', label=r'$\Delta p(t_k)_{wheels}$', )
        plt.plot(laser_d.distance.time, laser_d.distance.dist, 'r', label="$\Delta p(t_k)_{real}$")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.xlabel(r'$t_k$')
        plt.ylabel(r'$\Delta \theta(t_k)$')
        plt.title('Displacement: orientation')
        plt.plot(odometry_d.rotation.time, odometry_d.rotation.theta, 'b', label=r'$\Delta \theta(t_k)_{wheels}$')
        plt.plot(laser_d.rotation.time, laser_d.rotation.theta, 'r', label=r'$\Delta \theta(t_k)_{real}$')
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.xlabel(r'$t_k$')
        plt.ylabel(r'$\delta p(t_k)$')
        plt.title('Difference: position')
        plt.plot(threshold_position.time, threshold_position.dist, 'y', label=r'$\eta_p(t_k)$ - threshold')
        plt.plot(position_difference.time, position_difference.dif, 'g', label=r'$\delta p(t_k)$ - difference ')
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.xlabel(r'$t_k$')
        plt.ylabel(r'$\delta \theta(t_k)$')
        plt.title('Difference: orientation')
        plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y', label=r'$\eta_{\theta}(t_k)$ - threshold')
        plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label=r'$\delta \theta(t_k)$ - difference')
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.show()

        rospy.signal_shutdown('shutting down...')


def main():
    TractionDetector = traction_detector()
    try:
        TractionDetector.start_detection()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
