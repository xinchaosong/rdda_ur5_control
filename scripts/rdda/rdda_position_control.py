#!/usr/bin/env python

import sys
import rospy

from trajectory_msgs.msg import JointTrajectoryPoint

joint_pub = rospy.Publisher("rdda_interface/joint_cmds", JointTrajectoryPoint, queue_size=1)


def set_positions(position0, position1):
    rospy.init_node('rdda_position_control', anonymous=True)
    rate = rospy.Rate(20)

    for i in xrange(10):
        joint_cmds_msg = JointTrajectoryPoint()
        joint_cmds_msg.positions = (position0, position1)
        joint_pub.publish(joint_cmds_msg)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) == 3:
        m_pos_0 = float(sys.argv[1])
        m_pos_1 = float(sys.argv[2])

        set_positions(m_pos_0, m_pos_1)
