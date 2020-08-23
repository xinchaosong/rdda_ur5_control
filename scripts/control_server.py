#!/usr/bin/env python
import ast
import socket

from rdda_ur5_control.srv import NoParam, NoParamResponse, \
    Move, MoveResponse, MoveRead, MoveReadResponse, MoveTraj, MoveTrajResponse, MoveLinear, MoveLinearResponse, \
    SetRddaParam, SetRddaParamResponse, RddaData, RddaDataResponse

import rospy
from control_core import RddaUr5ControlCore


class RddaUr5ControlServer(object):
    """
    This class provides services for controlling RDDA-UR5/UR5e robot thought both ROS and UDP socket.
    """

    def __init__(self):
        rospy.init_node('rdda_ur5_control_server', anonymous=False)
        self.control_core = RddaUr5ControlCore()
        self.udp_recv_port = 56800
        self.udp_sent_port = 56801

    def start(self, udp_interface=False):
        try:
            if not rospy.has_param('rdda_ur5_control'):
                raise OSError("Failed to load the controlling parameters.")

            if not self.control_core.start():
                raise OSError("Stopped.")

            # RDDA services
            rospy.Service('rdda_ur5_control/set_rdda_stiffness', SetRddaParam, self.__set_rdda_stiffness)
            rospy.Service('rdda_ur5_control/init_rdda_stiffness', NoParam, self.__init_rdda_stiffness)
            rospy.Service('rdda_ur5_control/set_rdda_positions', SetRddaParam, self.__set_rdda_positions)
            rospy.Service('rdda_ur5_control/set_rdda_max_velocities', SetRddaParam, self.__set_rdda_max_velocities)
            rospy.Service('rdda_ur5_control/set_rdda_max_efforts', SetRddaParam, self.__set_rdda_max_efforts)
            rospy.Service('rdda_ur5_control/home_rdda', NoParam, self.__home_rdda)
            rospy.Service('rdda_ur5_control/read_rdda_positions', RddaData, self.__read_rdda_positions)
            rospy.Service('rdda_ur5_control/read_rdda_lower_bounds', RddaData, self.__read_rdda_lower_bounds)
            rospy.Service('rdda_ur5_control/read_rdda_upper_bounds', RddaData, self.__read_rdda_upper_bounds)
            rospy.Service('rdda_ur5_control/read_rdda_origins', RddaData, self.__read_rdda_origins)

            # UR5 services
            rospy.Service('rdda_ur5_control/move_ur5', Move, self.__move_ur5)
            rospy.Service('rdda_ur5_control/move_ur5_trajectory', MoveTraj, self.__move_ur5_trajectory)
            rospy.Service('rdda_ur5_control/move_ur5_linear', MoveLinear, self.__move_ur5_linear)
            rospy.Service('rdda_ur5_control/stop_ur5', NoParam, self.__stop_ur5)
            rospy.Service('rdda_ur5_control/home_ur5', NoParam, self.__home_ur5)

            # RDDA and UR5 combined services
            rospy.Service('rdda_ur5_control/move_read_discrete', MoveRead, self.__move_read_discrete)
            rospy.Service('rdda_ur5_control/move_read_continuous', MoveRead, self.__move_read_continuous)

            print "\nReady to process commands.\n"

            if udp_interface:
                self.__start_udp_interface()
            else:
                rospy.spin()

        except rospy.ROSInterruptException:
            return

    def __start_udp_interface(self):
        udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_server.bind(("", self.udp_recv_port))

        while not rospy.is_shutdown():
            command_str, addr = udp_server.recvfrom(128)
            rospy.loginfo(command_str)

            command = command_str.split(',')

            if command[0] == 'set_rdda_stiffness':
                result = self.control_core.set_rdda_stiffness(float(command[1]), float(command[2]))

            elif command[0] == 'init_rdda_stiffness':
                stiff0_default = rospy.get_param("/rdda_ur5_control/stiff_default/f0")
                stiff1_default = rospy.get_param("/rdda_ur5_control/stiff_default/f1")
                result = self.control_core.set_rdda_stiffness(stiff0_default, stiff1_default)

                print "RDDA stiffness has been initialized."

            elif command[0] == 'set_rdda_positions':
                result = self.control_core.set_rdda_positions(float(command[1]), float(command[2]))

            elif command[0] == 'set_rdda_max_velocities':
                result = self.control_core.set_rdda_max_velocities(float(command[1]), float(command[2]))

            elif command[0] == 'set_rdda_max_efforts':
                result = self.control_core.set_rdda_max_efforts(float(command[1]), float(command[2]))

            elif command[0] == 'home_rdda':
                result = self.control_core.home_rdda()

            elif command[0] == 'read_rdda_positions':
                result = self.control_core.read_rdda_positions()

            elif command[0] == 'read_rdda_lower_bounds':
                result = self.control_core.read_rdda_lower_bounds()

            elif command[0] == 'read_rdda_upper_bounds':
                result = self.control_core.read_rdda_upper_bounds()

            elif command[0] == 'read_rdda_origins':
                result = self.control_core.read_rdda_origins()

            elif command[0] == 'move_ur5':
                result = self.control_core.move_ur5(float(command[1]), float(command[2]), float(command[3]),
                                                    float(command[4]), float(command[5]), float(command[6]),
                                                    float(command[7]))

            elif command[0] == 'move_ur5_trajectory':
                result = self.control_core.move_ur5_trajectory(float(command[1]), int(command[2]), float(command[3]),
                                                               int(command[4]))

            elif command[0] == 'move_ur5_linear':
                result = self.control_core.move_ur5_linear(int(command[1]), float(command[2]), float(command[3]))

            elif command[0] == 'stop_ur5':
                result = self.control_core.stop_ur5()

            elif command[0] == 'set_ur5_joints':
                low_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/low")
                result = self.control_core.set_ur5_joints(float(command[1]), float(command[2]), float(command[3]),
                                                          float(command[4]), float(command[5]), float(command[6]),
                                                          low_velocity)

            elif command[0] == 'home_ur5':
                high_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/high")
                result = self.control_core.home_ur5(high_velocity)

            elif command[0] == 'move_read_discrete':
                low_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/low")
                result = self.control_core.move_read_discrete(float(command[1]), int(command[2]), low_velocity)
                print result

            elif command[0] == 'move_read_continuous':
                low_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/low")
                result = self.control_core.move_read_continuous(float(command[1]), int(command[2]), low_velocity)

            else:
                result = None

            udp_server.sendto(str(result), ('<broadcast>', self.udp_sent_port))

    def __set_rdda_stiffness(self, req):
        """
        Sets the RDDA stiffness.

        :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
        :return: SetRddaParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.set_rdda_stiffness(req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(return_code)

    def __init_rdda_stiffness(self, req):
        """
        Initializes the RDDA stiffness.

        :param req: NoParam: N/A
        :return: NoParamResponse: int8 return_code (0 or 1)
        """

        stiff0_default = rospy.get_param("/rdda_ur5_control/stiff_default/f0")
        stiff1_default = rospy.get_param("/rdda_ur5_control/stiff_default/f1")

        return_code = self.control_core.set_rdda_stiffness(stiff0_default, stiff1_default)

        print "RDDA stiffness has been initialized."

        return NoParamResponse(return_code)

    def __set_rdda_positions(self, req):
        """
        Sets the RDDA positions.

        :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
        :return: SetRddaParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.set_rdda_positions(req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(return_code)

    def __set_rdda_max_velocities(self, req):
        """
        Sets the RDDA max velocities.

        :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
        :return: SetRddaParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.set_rdda_max_velocities(req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(return_code)

    def __set_rdda_max_efforts(self, req):
        """
        Sets the RDDA max efforts.

        :param req: SetRddaParam: float64 finger0_param, float64 finger1_param
        :return: SetRddaParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.set_rdda_max_efforts(req.finger0_param, req.finger1_param)

        return SetRddaParamResponse(return_code)

    def __home_rdda(self, req):
        """
        Homes the RDDA.

        :param req: NoParam: N/A
        :return: NoParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.home_rdda()

        return NoParamResponse(return_code)

    def __read_rdda_lower_bounds(self, req):
        """
        Reads the RDDA lower bounds.

        :param req: RddaData: N/A
        :return: RddaDataResponse: float64[] angle_data
        """

        angle_bounds = ast.literal_eval(self.control_core.read_rdda_lower_bounds())

        return RddaDataResponse(angle_bounds)

    def __read_rdda_upper_bounds(self, req):
        """
        Reads the RDDA upper bounds.

        :param req: RddaData: N/A
        :return: RddaDataResponse: float64[] angle_data
        """

        angle_bounds = ast.literal_eval(self.control_core.read_rdda_upper_bounds())

        return RddaDataResponse(angle_bounds)

    def __read_rdda_origins(self, req):
        """
        Reads the RDDA origins.

        :param req: RddaData: N/A
        :return: RddaDataResponse: float64[] angle_data
        """

        angle_origins = ast.literal_eval(self.control_core.read_rdda_origins())

        return RddaDataResponse(angle_origins)

    def __move_ur5(self, req):
        """
        Moves the UR5/UR5e.

        :param req: Move: float64 x, float64 y, float64 z, float64 velocity
        :return: MoveRespons: int8 return_code (0 or 1)
        """

        return_code = self.control_core.move_ur5(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, req.velocity)

        return MoveResponse(return_code)

    def __move_ur5_trajectory(self, req):
        """
        Moves the UR5/UR5e along with a line.

        :param req: MoveTraj: float64 step_size, int64 step_num, float64 velocity, int64 wait
        :return: MoveTrajResponse: int8 return_code (0 or 1)
        """
        return_code = self.control_core.move_ur5_trajectory(req.step_size, req.step_num, req.velocity, req.wait)

        return MoveTrajResponse(return_code)

    def __move_ur5_linear(self, req):
        """
        Moves the UR5/UR5e along with a line.

        :param req: MoveLinear: float64 target
        :return: MoveLinearResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.move_ur5_linear(req.axis, req.target, req.velocity)

        return MoveLinearResponse(return_code)

    def __stop_ur5(self, req):
        """
        Stops the UR5/UR5e.

        :param req: NoParam: N/A
        :return: NoParamResponse: int8 return_code (0 or 1)
        """

        return_code = self.control_core.stop_ur5()

        return NoParamResponse(return_code)

    def __home_ur5(self, req):
        """
        Homes the UR5/UR5e.

        :param req: NoParam: N/A
        :return: NoParamResponse: int8 return_code (0 or 1)
        """

        high_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/high")
        return_code = self.control_core.home_ur5(high_velocity)

        return NoParamResponse(return_code)

    def __move_read_discrete(self, req):
        """
        Moves the UR5/UR5e and reads the RDDA data discretely.

        :param req: MoveRead: float64 velocity, int64 step_num
        :return: MoveReadResponse: float64[] angle_data
        """

        low_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/low")
        angle_data = self.control_core.move_read_discrete(req.step_size, req.step_num, low_velocity)

        return MoveReadResponse(angle_data)

    def __move_read_continuous(self, req):
        """
        Moves the UR5/UR5e and reads the RDDA data continuously.

        :param req: MoveRead: float64 velocity, int64 step_num
        :return: MoveReadResponse: float64[] angle_data
        """

        low_velocity = rospy.get_param("/rdda_ur5_control/velocity_default/low")
        angle_series = self.control_core.move_read_continuous(req.step_size, req.step_num, low_velocity)

        return MoveReadResponse(angle_series)

    def __read_rdda_positions(self, req):
        """
        Reads the RDDA positions.

        :param req: RddaData: N/A
        :return: RddaDataResponse: float64[] angle_data
        """

        angle = ast.literal_eval(self.control_core.read_rdda_positions())

        return RddaDataResponse(angle)


if __name__ == "__main__":
    m_rdda_ur5_control_server = RddaUr5ControlServer()
    m_rdda_ur5_control_server.start(udp_interface=True)
