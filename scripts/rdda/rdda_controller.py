#!/usr/bin/env python

import subprocess

from util import convert_str_to_float_list


def set_stiffness_proxy(stiff0, stiff1):
    command = "export ROS_MASTER_URI=http://ZBox:11311 " \
              "&& rosservice call /rdda_interface/set_stiff [%s,%s]" % (stiff0, stiff1)
    subprocess.check_call(command, shell=True)


def set_positions_proxy(position0, position1):
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosrun rdda_ur5_control rdda_position_control.py %s %s" % (position0, position1)
    subprocess.check_call(m_command, shell=True)


def homing_proxy():
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosrun rdda_interface homing.py"
    subprocess.check_call(m_command, shell=True)


def get_lower_bounds_proxy():
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosparam get /rdda_interface/lower_bounds"
    lower_bounds_text = str(subprocess.check_output(m_command, shell=True))
    lower_bounds = convert_str_to_float_list(lower_bounds_text)

    return lower_bounds


def get_upper_bounds_proxy():
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosparam get /rdda_interface/upper_bounds"
    upper_bounds_text = str(subprocess.check_output(m_command, shell=True))
    upper_bounds = convert_str_to_float_list(upper_bounds_text)

    return upper_bounds


def get_origins_proxy():
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosparam get /rdda_interface/origins"
    origins_text = str(subprocess.check_output(m_command, shell=True))
    origins = convert_str_to_float_list(origins_text)

    return origins


def get_positions_proxy(read_num=1):
    m_command = "export ROS_MASTER_URI=http://ZBox:11311 " \
                "&& rosrun rdda_ur5_control rdda_data_receiver.py %s" % read_num
    positions_str = str(subprocess.check_output(m_command, shell=True))
    angles = convert_str_to_float_list(positions_str)

    return angles
