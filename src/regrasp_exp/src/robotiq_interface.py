#!/usr/bin/env python

""" robotiq_interface.py
    script used to define a simple and easy interface with Robotiq 2f-85
    author: Michael Anres Lin (michaelv03@gmail.com)
    date: 11/14/2019
"""

import os
import sys
import roslib; roslib.load_manifest('ur_driver')
import rospy


class RobotiqInterface:
    """ An interface class for Robotiq 2F-85 """

    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        self.goal_state_publisher = rospy.Publisher('/rviz/moveit/update_custom_goal_state',
                                                        moveit_msgs.msg.RobotState,
                                                        queue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_max_velocity_scaling_factor(0.1)
        print "============ Set a max acceleration value of 0.1"
        print "============ Set a max velocity value of 0.1"

