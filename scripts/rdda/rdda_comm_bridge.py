#!/usr/bin/env python

import os
import subprocess
import sys
import socket

import rosnode
import rospy
from rdda_ros_controller import RddaController
from sensor_msgs.msg import JointState

rdda_proxy_control_in_port = 56802
rdda_proxy_control_out_port = 56803
rdda_repeater_out_port = 56804


def rdda_joint_states_repeater(data):
    states_repeater = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    states_repeater.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # string[] name
    # float64[] position
    # float64[] velocity
    # float64[] effort

    # rdda_states.append([data.position, data.velocity, data.effort])

    message = str(data.position)
    states_repeater.sendto(message, ('<broadcast>', rdda_repeater_out_port))
    rospy.loginfo(message)

    states_repeater.close()


def rdda_comm_bridge():
    rospy.init_node('rdda_comm_bridge', anonymous=False)
    rospy.Subscriber('/rdda_interface/joint_states', JointState, rdda_joint_states_repeater)
    control_proxy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    control_proxy.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    control_proxy.bind(("", rdda_proxy_control_in_port))
    rdda_controller = RddaController()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            command_str, addr = control_proxy.recvfrom(128)
        except socket.error, e:
            print str(e)
            break

        rospy.loginfo(command_str)
        command = command_str.split(',')

        if command[0] == 'set_rdda_stiffness':
            result = rdda_controller.set_stiffness((float(command[1]), float(command[2])))
        elif command[0] == 'set_rdda_positions':
            result = rdda_controller.set_positions((float(command[1]), float(command[2])))
        elif command[0] == 'set_rdda_max_velocities':
            result = rdda_controller.set_max_velocities((float(command[1]), float(command[2])))
        elif command[0] == 'set_rdda_max_efforts':
            result = rdda_controller.set_max_efforts((float(command[1]), float(command[2])))
        elif command[0] == 'home_rdda':
            result = rdda_controller.homing()
            rospy.set_param('/rdda_interface/origins', rdda_controller.joint_origins)
            rospy.set_param('/rdda_interface/upper_bounds', rdda_controller.joint_upper_bounds)
            rospy.set_param('/rdda_interface/lower_bounds', rdda_controller.joint_lower_bounds)
        elif command[0] == 'read_rdda_lower_bounds':
            result = rospy.get_param("/rdda_interface/lower_bounds")
        elif command[0] == 'read_rdda_upper_bounds':
            result = rospy.get_param("/rdda_interface/upper_bounds")
        elif command[0] == 'read_rdda_origins':
            result = rospy.get_param("/rdda_interface/origins")
        else:
            result = None

        rospy.loginfo(result)
        control_proxy.sendto(str(result), ('<broadcast>', rdda_proxy_control_out_port))

        rate.sleep()

    control_proxy.close()
    sys.exit(0)


if __name__ == '__main__':
    os.environ["ROS_MASTER_URI"] = "http://ZBox:11311"

    while "/rdda_comm_bridge" not in rosnode.get_node_names():
        try:
            rdda_comm_bridge()

        except rospy.ROSInterruptException:
            sys.exit(0)
