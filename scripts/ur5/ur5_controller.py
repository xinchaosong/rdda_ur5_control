#!/usr/bin/env python

import sys
import math

import rospy
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """

    if type(goal) is list:
        for index in xrange(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class Ur5Controller(object):
    """
    This class is for controlling UR5 robotic arm.
    """

    def __init__(self):
        super(Ur5Controller, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.home_joint_state = [0, - math.pi / 2, math.pi / 2, 0, math.pi / 2, math.pi]
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

    def homing(self, velocity):
        self.set_joint_state(self.home_joint_state, velocity)

    def get_robot_state(self):
        return self.robot.get_current_state()

    def get_robot_pose(self):
        return self.group.get_current_pose().pose

    def set_robot_pose(self, cartesian_goal, orientation_goal, velocity, wait=True):
        pose_goal = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(orientation_goal[0], orientation_goal[1], orientation_goal[2])

        pose_goal.position.x = cartesian_goal[0]
        pose_goal.position.y = cartesian_goal[1]
        pose_goal.position.z = cartesian_goal[2]
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]

        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_pose_target(pose_goal)
        self.group.go(wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose

        print self.get_joint_state()

        return all_close(pose_goal, current_pose, 0.01)

    def get_pose(self):
        return geometry_msgs.msg.Pose()

    def get_joint_state(self):
        return self.group.get_current_joint_values()

    def set_joint_state(self, angle_goal, velocity):
        if len(angle_goal) != 6:
            raise ValueError("Must provide the goal angles for all the 6 joints on the robot.")

        if not all(- math.pi <= angle <= math.pi for angle in angle_goal):
            raise ValueError("Each joint can only move from -math.pi to math.pi.")

        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.go(angle_goal, wait=True)
        self.group.stop()

        current_joints = self.get_joint_state()

        return all_close(angle_goal, current_joints, 0.01)

    def get_cartesian_position(self):
        current_pose = self.group.get_current_pose().pose
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        return x, y, z

    def set_cartesian_position(self, cartesian_goal, velocity, wait=True):
        current_pose = self.group.get_current_pose().pose

        return self.set_robot_pose(cartesian_goal, current_pose.orientation, velocity, wait)

    def get_orientation(self):
        current_pose = self.group.get_current_pose().pose
        quat = current_pose.orientation

        euler = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

        return euler

    def set_orientation(self, orientation, velocity, wait=True):
        current_pose = self.group.get_current_pose().pose

        return self.set_robot_pose(current_pose.position, orientation, velocity, wait)

    def plan_cartesian_path(self, way_points, step, velocity):
        current_robot_state = self.robot.get_current_state()

        (original_plan, fraction) = self.group.compute_cartesian_path(
            waypoints=way_points,
            eef_step=step,
            jump_threshold=0.0)

        plan = self.group.retime_trajectory(ref_state_in=current_robot_state, traj_in=original_plan,
                                            velocity_scaling_factor=velocity)

        return plan, fraction

    def execute_plan(self, plan, wait=True):
        self.group.execute(plan, wait=wait)

    def stop(self):
        self.group.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_controller', anonymous=True, disable_signals=True)
        m_ur5_controller = Ur5Controller()
        trial_position = [0.6, 0, 0.3]
        m_ur5_controller.homing(velocity=0.1)
        m_ur5_controller.set_cartesian_position(cartesian_goal=trial_position, velocity=0.2)
        m_ur5_controller.homing(velocity=0.1)

    except rospy.ROSInterruptException:
        sys.exit(1)

    except KeyboardInterrupt:
        sys.exit(1)
