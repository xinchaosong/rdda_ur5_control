#!/usr/bin/env python

import os
import ast
import copy
import subprocess
import time
import socket
from multiprocessing import Manager, Process

import rosnode
import rospy

from ur5.ur5_controller import Ur5Controller
from rdda.rdda_comm_bridge import rdda_proxy_control_in_port, rdda_proxy_control_out_port, rdda_repeater_out_port

robot_type = 'ur5e'
robot_ip = '192.168.0.101'
kinematics_config_file = '${HOME}/my_robot_calibration.yaml'


def terminal_run(command):
    subprocess.check_call(['/bin/bash', '-c', 'gnome-terminal -e "%s"' % command])


def init_moveit():
    if os.path.exists(kinematics_config_file):
        kinematics_config = ' kinematics_config:="%s"' % kinematics_config_file
    else:
        kinematics_config = ''

    try:
        while "/robot_state_publisher" not in rosnode.get_node_names():
            terminal_run('roslaunch ur_robot_driver %s_bringup.launch robot_ip:=%s%s'
                         % (robot_type, robot_ip, kinematics_config))
            time.sleep(4)

        while "/move_group" not in rosnode.get_node_names():
            terminal_run("roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch limited:=true")
            time.sleep(2)

        terminal_run("rosrun rdda_ur5_control rdda_comm_bridge.py")

        return True

    except subprocess.CalledProcessError:
        return False


def read_rdda_positions_multiprocessing(switch_shared, angle_series_shared):
    rdda_proxy_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    rdda_proxy_state.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    rdda_proxy_state.bind(("", rdda_repeater_out_port))

    while switch_shared.value == 1:
        data, addr = rdda_proxy_state.recvfrom(64)
        angles_str = data.decode()
        angles = ast.literal_eval(angles_str)
        angle_series_shared.append(angles[0])


class RddaUr5ControlCore(object):
    """
    This class provides the core methods for controlling RDDA-UR5/UR5e robot.
    """

    def __init__(self):
        self.ur5_controller = None
        self.udp_sent_port = rdda_proxy_control_in_port
        self.udp_recv_port = rdda_proxy_control_out_port
        self.rdda_proxy_control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rdda_proxy_control.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.rdda_proxy_control.bind(("", self.udp_recv_port))

    def start(self):
        try:
            moveit_running = init_moveit()

            if not moveit_running:
                raise OSError("Failed to connect to the hardware.")

            self.ur5_controller = Ur5Controller()

            return True

        except rospy.ROSInterruptException:
            return False

    def set_rdda_stiffness(self, stiff0, stiff1):
        """
        Sets the RDDA stiffness.

        :param stiff0: the stiffness of the finger #0
        :param stiff1: the stiffness of the finger #1
        :return: return code (0 or 1)
        """

        try:
            command_str = 'set_rdda_stiffness,%f,%f' % (stiff0, stiff1)
            self.__send_rdda_command(command_str.encode(), 16)

            print "RDDA stiffness has been set to %f %f." % (stiff0, stiff1)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def set_rdda_positions(self, position0, position1):
        """
        Sets the RDDA positions.

        :param position0: the position of the finger #0
        :param position1: the position of the finger #1
        :return: return code (0 or 1)
        """

        try:
            command_str = 'set_rdda_positions,%f,%f' % (position0, position1)
            self.__send_rdda_command(command_str.encode(), 16)

            print "RDDA positions have been set to %f %f." % (position0, position1)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def set_rdda_max_velocities(self, max_velocity0, max_velocity1):
        """
        Sets the RDDA max velocities.

        :param max_velocity0: the max velocity of the finger #0
        :param max_velocity1: the max velocity of the finger #1
        :return: return code (0 or 1)
        """

        try:
            command_str = 'set_rdda_max_velocities,%f,%f' % (max_velocity0, max_velocity1)
            self.__send_rdda_command(command_str.encode(), 16)

            print "RDDA max velocities have been set to %f %f." % (max_velocity0, max_velocity1)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def set_rdda_max_efforts(self, max_effort0, max_effort1):
        """
        Sets the RDDA max efforts.

        :param max_effort0: the max effort of the finger #0
        :param max_effort1: the max effort of the finger #1
        :return: return code (0 or 1)
        """

        try:
            command_str = 'set_rdda_max_efforts,%f,%f' % (max_effort0, max_effort1)
            self.__send_rdda_command(command_str.encode(), 16)

            print "RDDA max efforts have been set to %f %f." % (max_effort0, max_effort1)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def home_rdda(self):
        """
        Homes the RDDA.

        :return: return code (0 or 1)
        """

        try:
            command_str = 'home_rdda'
            self.__send_rdda_command(command_str.encode(), 16)
            print "RDDA positions have been homed."

            return 0

        except rospy.ROSInterruptException:
            return 1

    def read_rdda_positions(self):
        """
        Reads the RDDA positions.

        :return: a pair of float number representing the current positions of RDDA
        """
        try:
            rdda_positions_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            rdda_positions_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            rdda_positions_receiver.bind(("", rdda_repeater_out_port))
            data, addr = rdda_positions_receiver.recvfrom(64)
            angle = data.decode()

            print "RDDA positions are %s." % angle

            return angle

        except rospy.ROSInterruptException:
            return []

    def read_rdda_lower_bounds(self):
        """
        Reads the RDDA lower bounds.

        :return: a pair of float number representing the lower bounds of RDDA
        """

        try:
            command_str = 'read_rdda_lower_bounds'
            angle_bounds = self.__send_rdda_command(command_str, 64).decode()

            print "RDDA lower bounds are %s." % angle_bounds

            return angle_bounds

        except rospy.ROSInterruptException:
            return []

    def read_rdda_upper_bounds(self):
        """
        Reads the RDDA upper bounds.

        :return: a pair of float number representing the upper bounds of RDDA
        """

        try:
            command_str = 'read_rdda_upper_bounds'
            angle_bounds = self.__send_rdda_command(command_str.encode(), 64).decode()

            print "RDDA upper bounds are %s." % angle_bounds

            return angle_bounds

        except rospy.ROSInterruptException:
            return []

    def read_rdda_origins(self):
        """
        Reads the RDDA origins.

        :return: a pair of float number representing the origins of RDDA
        """

        try:
            command_str = 'read_rdda_origins'
            angle_origins = self.__send_rdda_command(command_str.encode(), 64).decode()

            print "RDDA origins are %s." % angle_origins

            return angle_origins

        except rospy.ROSInterruptException:
            return []

    def read_ur5_position(self):
        try:
            x, y, z = self.ur5_controller.get_cartesian_position()

            print "The current UR5 position is %f, %f, %f." % (x, y, z)

            return [x, y, z]

        except rospy.ROSInterruptException:
            return []

    def set_ur5_joints(self, j1, j2, j3, j4, j5, j6, velocity):
        try:
            self.ur5_controller.set_joint_state((j1, j2, j3, j4, j5, j6), velocity)

            final_x, final_y, final_z = self.ur5_controller.get_cartesian_position()
            final_roll, final_pitch, final_yaw = self.ur5_controller.get_orientation()

            print "UR5/UR5e has been moved to (%f, %f, %f, %f, %f, %f) with the velocity of %f." \
                  % (final_x, final_y, final_z, final_roll, final_pitch, final_yaw, velocity)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def move_ur5(self, x, y, z, roll, pitch, yaw, velocity):
        """
        Moves the UR5/UR5e.

        :param x:        target x
        :param y:        target y
        :param z:        target z
        :param roll:     target roll
        :param pitch:    target pitch
        :param yaw:      target yaw
        :param velocity: the moving velocity
        :return: return code (0 or 1)
        """

        cartesian_goal = (x, y, z)
        orientation_goal = (roll, pitch, yaw)

        try:
            self.ur5_controller.set_robot_pose(cartesian_goal=cartesian_goal, orientation_goal=orientation_goal,
                                               velocity=velocity)

            final_x, final_y, final_z = self.ur5_controller.get_cartesian_position()
            final_roll, final_pitch, final_yaw = self.ur5_controller.get_orientation()

            print "UR5/UR5e has been moved to (%f, %f, %f, %f, %f, %f) with the velocity of %f." \
                  % (final_x, final_y, final_z, final_roll, final_pitch, final_yaw, velocity)
            print self.ur5_controller.get_joint_state()

            return 0

        except rospy.ROSInterruptException:
            return 1

    def move_ur5_trajectory(self, step_size, step_num, velocity, wait):
        """
        Moves the UR5/UR5e along with a line.

        :param step_size: the size of each step
        :param step_num: the total number of the steps to move
        :param velocity: moving velocity
        :param wait: True if wait for the motion to finish, False otherwise
        :return: return code (0 or 1)
        """

        way_poses = []
        robot_pose = self.ur5_controller.get_robot_pose()

        try:
            if wait:
                print "UR5/UR5e is moving."

            for i in xrange(step_num):
                robot_pose.position.y += step_size
                way_poses.append(copy.deepcopy(robot_pose))

            plan, fraction = self.ur5_controller.plan_cartesian_path(way_points=way_poses, step=abs(step_size),
                                                                     velocity=velocity)

            self.ur5_controller.execute_plan(plan=plan, wait=wait)

            final_position = way_poses[-1].position

            if wait:
                print "UR5/UR5e moved to (%f, %f, %f)." % (final_position.x, final_position.y, final_position.z)
            else:
                print "UR5/UR5e is moving to (%f, %f, %f)." % (final_position.x, final_position.y, final_position.z)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def move_ur5_linear(self, axis, target, velocity, wait):
        """
        Moves the UR5/UR5e along with a line.

        :param axis: the axis to move along
        :param target: the target to move to
        :param velocity: moving velocity
        :param wait: if waits for the movement to finish for not before returns
        :return: return code (0 or 1)
        """

        way_poses = []

        # Adds the current pose as the source pose.
        robot_pose = self.ur5_controller.get_robot_pose()
        way_poses.append(copy.deepcopy(robot_pose))

        # Adds the target pose.
        if axis == 0:
            robot_pose.position.x = target
        elif axis == 1:
            robot_pose.position.y = target
        elif axis == 2:
            robot_pose.position.z = target

        way_poses.append(copy.deepcopy(robot_pose))

        try:
            if wait:
                print "UR5/UR5e is moving with a velocity of %f." % velocity

            plan, fraction = self.ur5_controller.plan_cartesian_path(way_points=way_poses, step=0.005,
                                                                     velocity=velocity)
            self.ur5_controller.execute_plan(plan=plan, wait=wait)
            final_x, final_y, final_z = self.ur5_controller.get_cartesian_position()

            if wait:
                print "UR5/UR5e moved to (%f, %f, %f)." % (final_x, final_y, final_z)
            else:
                print "UR5/UR5e is moving to (%f, %f, %f)." % (final_x, final_y, final_z)

            return 0

        except rospy.ROSInterruptException:
            return 1

    def stop_ur5(self):
        """
        Stops the UR5/UR5e.

        :return: return code (0 or 1)
        """

        try:
            self.ur5_controller.stop()
            print "UR5/UR5e has been stopped."

            return 0

        except rospy.ROSInterruptException:
            return 1

    def home_ur5(self, velocity):
        """
        Homes the UR5/UR5e.

        :return: return code (0 or 1)
        """

        try:
            self.ur5_controller.homing(velocity)
            print "UR5/UR5e has been moved back to the home pose."

            return 0

        except rospy.ROSInterruptException:
            return 1

    def move_read_discrete(self, step_size, step_num, velocity):
        """
        Moves the UR5/UR5e and reads the RDDA data discretely.

        :param step_size: the size of each step
        :param step_num: the total number of the steps to move
        :param velocity: moving velocity
        :return: a list of float number representing the position data of RDDA
        """

        angle_data = []
        x, y, z = self.ur5_controller.get_cartesian_position()

        try:
            for i in xrange(step_num):
                y += step_size
                self.ur5_controller.set_cartesian_position((x, y, z), velocity)
                angle = self.read_rdda_positions()
                angle_data.append(float(angle))

                print "UR5/UR5e moved to (%f, %f, %f)." % (x, y, z)

            print angle_data

            return angle_data

        except rospy.ROSInterruptException:
            return []

    def move_read_continuous(self, step_size, step_num, velocity):
        """
        Moves the UR5/UR5e and reads the RDDA data continuously.

        :param step_size: the size of each step
        :param step_num: the total number of the steps to move
        :param velocity: moving velocity
        :return: a list of float number representing the position data of RDDA
        """

        way_poses = []
        switch = Manager().Value('i', 1)
        angle_series = Manager().list()
        robot_pose = self.ur5_controller.get_robot_pose()
        read_multiprocessing = Process(target=read_rdda_positions_multiprocessing, args=(switch, angle_series))

        try:
            print "UR5/UR5e is moving."

            for i in xrange(step_num):
                robot_pose.position.y += step_size
                way_poses.append(copy.deepcopy(robot_pose))

            plan, fraction = self.ur5_controller.plan_cartesian_path(way_points=way_poses, step=abs(step_size),
                                                                     velocity=velocity)

            print plan

            read_multiprocessing.start()
            self.ur5_controller.execute_plan(plan=plan, wait=True)
            switch.value = 0
            read_multiprocessing.join()

            final_position = way_poses[-1].position

            print "UR5/UR5e moved to (%f, %f, %f)." % (final_position.x, final_position.y, final_position.z)
            print "Collected the data of %d RDDA positions." % len(angle_series)

            return angle_series

        except rospy.ROSInterruptException:
            return []

    def __send_rdda_command(self, command_str, recv_size):
        self.rdda_proxy_control.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.rdda_proxy_control.recvfrom(recv_size)

        return data


if __name__ == "__main__":
    m_rdda_ur5_control_core = RddaUr5ControlCore()
    m_rdda_ur5_control_core.start()
