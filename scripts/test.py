#!/usr/bin/env python
from time import sleep

import rospy
from rdda_ur5_control.srv import SetRddaParam, RddaData, Move, MoveTraj, NoParam

if __name__ == "__main__":
    rospy.wait_for_service('rdda_ur5_control/home_rdda')
    rospy.wait_for_service('rdda_ur5_control/set_rdda_stiffness')
    rospy.wait_for_service('rdda_ur5_control/read_rdda_positions')
    rospy.wait_for_service('rdda_ur5_control/move_ur5')
    rospy.wait_for_service('rdda_ur5_control/move_ur5_trajectory')
    rospy.wait_for_service('rdda_ur5_control/stop_ur5')
    rospy.wait_for_service('rdda_ur5_control/home_ur5')

    try:
        home_rdda = rospy.ServiceProxy('rdda_ur5_control/home_rdda', NoParam)
        set_rdda_stiffness = rospy.ServiceProxy('rdda_ur5_control/set_rdda_stiffness', SetRddaParam)
        read_rdda_positions = rospy.ServiceProxy('rdda_ur5_control/read_rdda_positions', RddaData)
        move_ur5 = rospy.ServiceProxy('rdda_ur5_control/move_ur5', Move)
        move_ur5_trajectory = rospy.ServiceProxy('rdda_ur5_control/move_ur5_trajectory', MoveTraj)
        stop_ur5 = rospy.ServiceProxy('rdda_ur5_control/stop_ur5', NoParam)
        home_ur5 = rospy.ServiceProxy('rdda_ur5_control/home_ur5', NoParam)

        home_ur5()
        move_ur5(0.6, -0.2, 0.2, 0.05)
        # base_angle = read_rdda_positions()
        # cur_angle = base_angle
        # set_rdda_stiffness(0.1, 0.1)
        #move_ur5_trajectory(0.01, 50, 0.05, 0)
        #
        # while abs(base_angle - cur_angle) < 0.1:
        #     cur_angle = read_rdda_positions()
        #
        sleep(4)
        stop_ur5()
        #move_ur5_trajectory(0.01, 50, 0.02, 1)
        home_ur5()

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
