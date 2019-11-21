#!/usr/bin/env python

import sys

import rospy
from sensor_msgs.msg import JointState

joint_states = []


def callback(data):
    """
    Receives the data from the ROS service "/rdda_interface/joint_states".

    :param data: string[] name (0,1)
                 float64[] position (0,1)
                 float64[] velocity (0,1)
                 float64[] effort (0,1)
    :return:
    """

    joint_states.append(data.position[0])


def get_joint_states(read_num=1):
    rospy.init_node('rdda_data_receiver', anonymous=True)
    rospy.Subscriber('/rdda_interface/joint_states', JointState, callback)

    while True:
        if len(joint_states) >= read_num:
            break

    return joint_states[:read_num]


if __name__ == '__main__':
    if len(sys.argv) == 2:
        m_read_num = int(sys.argv[1])

        print get_joint_states(m_read_num)
