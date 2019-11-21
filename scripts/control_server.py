#!/usr/bin/env python

import copy
import sys
from multiprocessing import Manager, Process

from rdda_ur5_control.srv import Move, MoveResponse, NoParam, NoParamResponse, MoveRead, MoveReadResponse, \
    SetRddaParam, SetRddaParamResponse, RddaData, RddaDataResponse
import rospy

from ur5.ur5_controller import Ur5Controller
from rdda import rdda_controller, rdda_data_receiver


def rdda_ur5_control_server():
    rospy.init_node('rdda_ur5_control_server')

    # RDDA services
    rospy.Service('rdda_ur5_control/set_rdda_stiffness', SetRddaParam, set_rdda_stiffness)
    rospy.Service('rdda_ur5_control/init_rdda_stiffness', NoParam, init_rdda_stiffness)
    rospy.Service('rdda_ur5_control/set_rdda_positions', SetRddaParam, set_rdda_positions)
    rospy.Service('rdda_ur5_control/home_rdda', NoParam, home_rdda)
    rospy.Service('rdda_ur5_control/read_rdda_positions', RddaData, read_rdda_positions)
    rospy.Service('rdda_ur5_control/read_rdda_lower_bounds', RddaData, read_rdda_lower_bounds)
    rospy.Service('rdda_ur5_control/read_rdda_upper_bounds', RddaData, read_rdda_upper_bounds)
    rospy.Service('rdda_ur5_control/read_rdda_origins', RddaData, read_rdda_origins)

    # UR5 services
    rospy.Service('rdda_ur5_control/move_ur5', Move, move_ur5)
    rospy.Service('rdda_ur5_control/home_ur5', NoParam, home_ur5)

    # RDDA and UR5 combined services
    rospy.Service('rdda_ur5_control/move_read_discrete', MoveRead, move_read_discrete)
    rospy.Service('rdda_ur5_control/move_read_continuous', MoveRead, move_read_continuous)

    print "\nReady to process commands.\n"

    rospy.spin()


def set_rdda_stiffness(req):
    """
    Sets the RDDA stiffness.

    :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
    :return: SetRddaParamResponse: int8 return_code (0 or 1)
    """
    try:
        rdda_controller.set_stiffness_proxy(req.finger0_param, req.finger1_param)

        print "RDDA stiffness has been set to %s %s." % (req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(0)

    except rospy.ROSInterruptException:
        return SetRddaParamResponse(1)
    except KeyboardInterrupt:
        return SetRddaParamResponse(1)


def init_rdda_stiffness(req):
    """
    Initializes the RDDA stiffness.

    :param req: NoParam: N/A
    :return: NoParamResponse: int8 return_code (0 or 1)
    """
    try:
        rdda_controller.set_stiffness_proxy(stiff0_default, stiff1_default)

        print "RDDA stiffness has been initialized to %s %s." % (stiff0_default, stiff1_default)

        return NoParamResponse(0)

    except rospy.ROSInterruptException:
        return NoParamResponse(1)
    except KeyboardInterrupt:
        return NoParamResponse(1)


def set_rdda_positions(req):
    """
    Sets the RDDA positions.

    :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
    :return: SetRddaParamResponse: int8 return_code (0 or 1)
    """
    try:
        rdda_controller.set_positions_proxy(req.finger0_param, req.finger1_param)

        print "RDDA positions have been set to %s %s." % (req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(0)

    except rospy.ROSInterruptException:
        return SetRddaParamResponse(1)
    except KeyboardInterrupt:
        return SetRddaParamResponse(1)


def home_rdda(req):
    """
    Homes the RDDA.

    :param req: NoParam: N/A
    :return: NoParamResponse: int8 return_code (0 or 1)
    """
    try:
        rdda_controller.homing_proxy()

        print "RDDA positions have been homed."

        return NoParamResponse(0)

    except rospy.ROSInterruptException:
        return NoParamResponse(1)
    except KeyboardInterrupt:
        return NoParamResponse(1)


def read_rdda_lower_bounds(req):
    """
    Reads the RDDA lower bounds.

    :param req: RddaData: N/A
    :return: RddaDataResponse: float64[] angle_data
    """
    try:
        angle_bounds = rdda_controller.get_lower_bounds_proxy()

        print "RDDA lower bounds are %s." % angle_bounds

        return RddaDataResponse(angle_bounds)

    except rospy.ROSInterruptException:
        return RddaDataResponse(1)
    except KeyboardInterrupt:
        return RddaDataResponse(1)


def read_rdda_upper_bounds(req):
    """
    Reads the RDDA upper bounds.

    :param req: RddaData: N/A
    :return: RddaDataResponse: float64[] angle_data
    """
    try:
        angle_bounds = rdda_controller.get_upper_bounds_proxy()

        print "RDDA upper bounds are %s." % angle_bounds

        return RddaDataResponse(angle_bounds)

    except rospy.ROSInterruptException:
        return RddaDataResponse(1)
    except KeyboardInterrupt:
        return RddaDataResponse(1)


def read_rdda_origins(req):
    """
    Reads the RDDA origins.

    :param req: RddaData: N/A
    :return: RddaDataResponse: float64[] angle_data
    """
    try:
        angle_origins = rdda_controller.get_origins_proxy()

        print "RDDA origins are %s." % angle_origins

        return RddaDataResponse(angle_origins)

    except rospy.ROSInterruptException:
        return RddaDataResponse(1)
    except KeyboardInterrupt:
        return RddaDataResponse(1)


def move_ur5(req):
    """
    Moves the UR5/UR5e.

    :param req: Move: float64 x, float64 y, float64 z, float64 velocity
    :return: MoveRespons: int8 return_code (0 or 1)
    """
    cartesian_goal = (req.x, req.y, req.z)

    try:
        ur5_controller.set_cartesian_position(cartesian_goal, req.velocity)

        print "UR5/UR5e has been moved to (%s, %s, %s) with the velocity of %s." % (req.x, req.y, req.z, req.velocity)

        return MoveResponse(0)

    except rospy.ROSInterruptException:
        return MoveResponse(1)
    except KeyboardInterrupt:
        return MoveResponse(1)


def home_ur5(req):
    """
    Homes the UR5/UR5e.

    :param req: NoParam: N/A
    :return: NoParamResponse: int8 return_code (0 or 1)
    """
    try:
        ur5_controller.homing(high_velocity)

        print "UR5/UR5e has been moved back to the home pose."

        return NoParamResponse(0)

    except rospy.ROSInterruptException:
        return NoParamResponse(1)
    except KeyboardInterrupt:
        return NoParamResponse(1)


def move_read_discrete(req):
    """
    Moves the UR5/UR5e and reads the RDDA data discretely.

    :param req: MoveRead: float64 velocity, int64 step_num
    :return: MoveReadResponse: float64[] angle_data
    """
    angle_data = []
    x, y, z = ur5_controller.get_cartesian_position()

    try:
        for i in xrange(req.step_num):
            y += req.velocity
            ur5_controller.set_cartesian_position((x, y, z), low_velocity)
            angle = rdda_controller.get_positions_proxy()[working_finger_index]
            angle_data.append(angle)

            print "UR5/UR5e moved to (%s, %s, %s)." % (x, y, z)
            print "RDDA position is %s." % angle

        return MoveReadResponse(angle_data)

    except rospy.ROSInterruptException:
        return MoveReadResponse([])
    except KeyboardInterrupt:
        return MoveReadResponse([])


def move_read_continuous(req):
    """
    Moves the UR5/UR5e and reads the RDDA data continuously.

    :param req: MoveRead: float64 velocity, int64 step_num
    :return: MoveReadResponse: float64[] angle_data
    """
    way_poses = []
    switch = Manager().Value('i', 1)
    angle_series = Manager().list()
    robot_pose = ur5_controller.get_robot_pose()
    read_multiprocessing = Process(target=_read_rdda_positions_multiprocessing, args=(switch, angle_series))

    try:
        print "UR5/UR5e is moving."

        for i in xrange(req.step_num):
            robot_pose.position.y += req.velocity
            way_poses.append(copy.deepcopy(robot_pose))

        plan, fraction = ur5_controller.plan_cartesian_path(way_points=way_poses, step=req.velocity,
                                                            velocity=req.velocity)

        read_multiprocessing.start()
        ur5_controller.execute_plan(plan)
        switch.value = 0
        read_multiprocessing.join()

        final_position = way_poses[-1].position

        print "UR5/UR5e moved to (%s, %s, %s)." % (final_position.x, final_position.y, final_position.z)
        print "Collected the data of %s RDDA positions." % len(angle_series)

        return MoveReadResponse(angle_series)

    except rospy.ROSInterruptException:
        return MoveReadResponse([])
    except KeyboardInterrupt:
        return MoveReadResponse([])


def read_rdda_positions(req):
    """
    Reads the RDDA positions.

    :param req: RddaData: N/A
    :return: RddaDataResponse: float64[] angle_data
    """
    try:
        angle = rdda_controller.get_positions_proxy()

        print "RDDA position is %s." % angle

        return RddaDataResponse(angle)

    except rospy.ROSInterruptException:
        return RddaDataResponse(-999.0)
    except KeyboardInterrupt:
        return RddaDataResponse(-999.0)


def _read_rdda_positions_multiprocessing(switch_shared, angle_series_shared):
    while switch_shared.value == 1:
        angle = rdda_controller.get_positions_proxy(5)
        angle_series_shared += angle


if __name__ == "__main__":
    high_velocity = 0.1
    low_velocity = 0.02
    stiff0_default = 0.1
    stiff1_default = 0.1
    working_finger_index = 0

    try:
        ur5_controller = Ur5Controller()
        rdda_ur5_control_server()
    except rospy.ROSInterruptException:
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(1)
