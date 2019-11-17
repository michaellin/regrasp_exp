#!/usr/bin/env python

import rospy
from regrasp_exp.srv import logData, logDataResponse
from ur5_interface import UR5Interface
from robotiq_interface import RobotiqInterface
import os, time
import numpy as np

# pregrasp pose
PREGRASP_POSE = np.array([1.442132192579322, -1.81151146845066, 2.301598803677338, -2.057281855307231, -1.5725453642862393, 1.4418938000460932])
START_POSE = np.array([1.4435208222959686, -1.797202303913668, 1.9166961218088687, -2.7331856205814793, -1.4603545035395626, 1.5049200971698213])
def mainLoop():
    global dl_service
    global ur5

    rospy.init_node('regraspExp', disable_signals=True)

    # Make this node a service
    dl_service = rospy.ServiceProxy('logData', logData)

    # Initialize ur5 interface
    ur5 = UR5Interface()

    ur5.goto_home_pose()

    time.sleep(2)

    # Initialize robotiq interface
    gripper = RobotiqInterface()

    # Run data logger process
    os.system("rosrun regrasp_exp exp_data_log.py &")
    time.sleep(4)

    # Start logging data
    resp = dl_service('Start')

    ur5.goto_joint_target(PREGRASP_POSE, wait=False)

    time.sleep(2)

    while not rospy.is_shutdown():

        gripper.goto_gripper_pos(180)

        time.sleep(1)

        ur5.goto_home_pose(wait=False)

        time.sleep(1)

        ur5.goto_joint_target(START_POSE, wait=False)

        time.sleep(1)

        gripper.goto_gripper_pos(120)

        time.sleep(10)

        ur5.goto_home_pose(wait=False)

        time.sleep(1)

        gripper.goto_gripper_pos(180)

        time.sleep(1)

        ur5.goto_joint_target(PREGRASP_POSE, wait=False)

        gripper.goto_gripper_pos(0)

        time.sleep(1)


        #while not rospy.is_shutdown():

            # command trajectories


if __name__ == '__main__':
    global dl_service
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        resp = dl_service('Stop')
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    except Exception:
        resp = dl_service('Stop')
        print("other exceptions")
