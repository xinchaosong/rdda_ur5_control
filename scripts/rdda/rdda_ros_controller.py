import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rdda_interface.msg import ControlState

from rdda_interface.srv import SetStiffness
from rdda_interface.srv import SetMaxVelocity
from rdda_interface.srv import SetMaxEffort

import time
import numpy as np


class RddaController:

    def __init__(self):
        # self.joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size=1)
        # self.joint_sub = rospy.Subscriber("rdd/joint_states", JointStates, self.subjointstates_callback)
        self.joint_pub = rospy.Publisher("rdda_interface/joint_cmds", JointTrajectoryPoint, queue_size=1)
        self.joint_sub = rospy.Subscriber("rdda_interface/joint_states", JointState, self.subjointstates_callback)
        self.ctrl_sub = rospy.Subscriber("rdda_interface/ctrl_states", ControlState, self.subctrlstates_callback)

        self.has_states_msg = False
        self.has_ctrl_msg = False

        """ Joint states """
        self.actual_positions = [0.0, 0.0]
        self.actual_velocities = [0.0, 0.0]
        self.external_efforts = [0.0, 0.0]
        self.applied_efforts = [0.0, 0.0]
        self.ts_nsec = 0.0
        self.ts_sec = 0.0

        self.joint_upper_bounds = [0.0, 0.0]
        self.joint_lower_bounds = [0.0, 0.0]
        self.joint_origins = [0.0, 0.0]

    def subjointstates_callback(self, joint_states_msg):
        self.has_states_msg = True
        self.actual_positions = joint_states_msg.position
        self.actual_velocities = joint_states_msg.velocity
        self.external_efforts = joint_states_msg.effort
        """ Ignore applied effort and time from ControlState. """
        # self.ts_nsec = JointStates_msg.header

    def subctrlstates_callback(self, ctrl_states_msg):
        self.has_ctrl_msg = True
        self.applied_efforts = ctrl_states_msg.applied_effort

    """ Publish additive positions in the loop. """
    def set_positions(self, positions=(0.0, 0.0)):
        joint_cmds_msg = JointTrajectoryPoint()
        joint_cmds_msg.positions = positions
        self.joint_pub.publish(joint_cmds_msg)

    def set_stiffness(self, stiffness=(1.0, 1.0)):
        rospy.wait_for_service('/rdda_interface/set_stiff')
        try:
            set_stiff = rospy.ServiceProxy('/rdda_interface/set_stiff', SetStiffness)
            res = set_stiff(stiffness)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_max_velocities(self, max_vel=(5.0, 5.0)):
        rospy.wait_for_service('/rdda_interface/set_max_vel')
        try:
            set_max_vel = rospy.ServiceProxy('/rdda_interface/set_max_vel', SetMaxVelocity)
            res = set_max_vel(max_vel)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
    def set_max_efforts(self, max_eff=(5.0, 5.0)):
        rospy.wait_for_service('/rdda_interface/set_max_eff')
        try:
            set_max_eff = rospy.ServiceProxy('rdda_interface/set_max_eff', SetMaxEffort)
            res = set_max_eff(max_eff)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # def publish_joint_cmds(self, pos_ref=(0.0, 0.0), vel_sat=(5.0, 5.0),
    # tau_sat=(5.0, 5.0), stiffness=(0.0, 0.0), freq_anti_alias=500.0):
        # joint_cmd_msg = JointCommands()
        # JointCommands_msg = JointTrajectoryPoint()
        # JointCommands_msg.positions
        # joint_cmd_msg.pos_ref = pos_ref
        # joint_cmd_msg.vel_sat = vel_sat
        # joint_cmd_msg.tau_sat = tau_sat
        # joint_cmd_msg.stiffness = stiffness
        # joint_cmd_msg.freq_anti_alias = freq_anti_alias

        # self.joint_pub.publish(JointCommands_msg)

    """ Fingers return to origin with arbitrary initial conditions. 
        Note: This functions is allowed only when fingers are centered manually. """
    def homing_trivial(self):

        """Make sure ROS message received"""
        while not self.has_states_msg:
            rospy.sleep(0.01)

        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([10, 10])
        rate = rospy.Rate(20)
        time_interval = 0.0
        tau_threshold = np.array([0.14, 0.14])
        done_finger0 = False
        done_finger1 = False
        done = False

        self.set_stiffness(stiffness=stiffness)
        while not done and not rospy.is_shutdown():
            time_interval += 1e-5
            # pos_ref[0] = -1.0 * np.sin(time_interval)
            tau_measured = self.external_efforts

            if not done_finger0:
                if tau_measured[0] > tau_threshold[0]:
                    pos_ref[0] += 1.27
                    done_finger0 = True
                else:
                    pos_ref[0] += -1.0 * time_interval
            if not done_finger1:
                if tau_measured[1] > tau_threshold[1]:
                    pos_ref[1] += 1.15
                    done_finger1 = True
                else:
                    pos_ref[1] += -1.0 * time_interval

            # self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
            self.set_positions(positions=pos_ref)

            # rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
            rate.sleep()
            done = done_finger0 and done_finger1
        time.sleep(0.5)

    """ Each finger detect two hardstops then go back to origin. 
        Note: This function is used to center the fingers and record the origins & hard-stops. """
    def homing(self):

        """Make sure ROS message received."""
        while not self.has_states_msg:
            rospy.sleep(0.01)

        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([10, 10])
        cmd_tau_measured = np.array([0.0, 0.0])
        lower_threshold = 0.3
        upper_threshold = 0.6
        collision_threshold = 0.3
        check_upper_bounds = [False, False]
        check_lower_bounds = [False, False]
        check_origins = [False, False]
        rate = rospy.Rate(20)
        time_interval = 0.05
        loop_num = 0

        """ Initialize fingers' stiffness before homing routine. """
        self.set_stiffness(stiffness=stiffness)

        """ Step1: Record the lower bounds of both fingers. """
        while not rospy.is_shutdown() and not (check_lower_bounds[0] and check_lower_bounds[1]):
            loop_num += 1
            # pos_measured = self.act_pos
            pos_measured = self.actual_positions
            if loop_num > 10:
                cmd_tau_measured = self.applied_efforts
            for i in range(2):
                if not check_lower_bounds[i]:
                    if np.absolute(cmd_tau_measured[i]) > lower_threshold:
                        self.joint_lower_bounds[i] = pos_measured[i]
                        check_lower_bounds[i] = True
                    else:
                        pos_ref[i] += -1.0 * time_interval
            self.set_positions(positions=pos_ref)
            rate.sleep()
        time.sleep(0.5)

        """ Step2: Record upper bound of each finger. """
        cmd_tau_measured = [0.0, 0.0]
        for i in range(2):
            loop_num = 0
            while not rospy.is_shutdown() and not check_upper_bounds[i]:
                loop_num += 1
                pos_measured = self.actual_positions
                if loop_num > 10:
                    cmd_tau_measured = self.applied_efforts
                if np.absolute(cmd_tau_measured[i]) > upper_threshold and \
                        np.absolute(pos_measured[i]-self.joint_lower_bounds[i]) > 0.8:
                    self.joint_upper_bounds[i] = pos_measured[i]
                    check_upper_bounds[i] = True
                else:
                    pos_ref[i] += time_interval
                self.set_positions(positions=pos_ref)
                rate.sleep()
            pos_ref[i] = self.joint_lower_bounds[i]
            self.set_positions(positions=pos_ref)
            rate.sleep()
            time.sleep(1)
        time.sleep(1)

        """ Step3: Close fingers then record origin when colliding. """
        cmd_tau_measured = [0.0, 0.0]
        loop_num = 0
        while not rospy.is_shutdown() and not (check_origins[0] and check_origins[1]):
            loop_num += 1
            pos_measured = self.actual_positions
            if loop_num > 20:
                cmd_tau_measured = self.applied_efforts
            for i in range(2):
                if not check_origins[i]:
                    if np.absolute(cmd_tau_measured[i]) > collision_threshold and np.absolute(pos_measured[i]-self.joint_lower_bounds[i]>0.2):
                        self.joint_origins[i] = pos_measured[i]
                        check_origins[i] = True
                    else:
                        pos_ref[i] += time_interval
            self.set_positions(positions=pos_ref)
            rate.sleep()
        rospy.loginfo("lower[{}, {}], upper: [{}, {}], origin: [{} {}]".format(self.joint_lower_bounds[0], self.joint_lower_bounds[1],
                                                                               self.joint_upper_bounds[0], self.joint_upper_bounds[1],
                                                                               self.joint_origins[0], self.joint_origins[1]))
        time.sleep(0.5)

    """Sinusoid wave for position tests on finger 0. 
        Finger will start at current position, make sure enough space to move."""
    def harmonic_wave(self):
        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([5.0, 5.0])
        vel_sat = (5.0, 5.0)
        rate = rospy.Rate(20)
        time_interval = 0.0

        self.set_stiffness(stiffness=stiffness)
        self.set_max_velocities(max_vel=vel_sat)
        while not rospy.is_shutdown():
            time_interval += 0.05
            pos_ref[0] = -0.5 * np.sin(time_interval)
            self.set_positions(positions=pos_ref)
            rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
            rate.sleep()
