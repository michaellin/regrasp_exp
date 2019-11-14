#!/usr/bin/env python

import rospy
from regrasp_exp.srv import logData, logDataResponse
from ur5_interface import UR5Interface
import os, time
import numpy as np

# pregrasp pose
PREGRASP_POSE = np.array([1.6839803833708726, -1.765108549617132, 2.217964419827103, -2.0200764935285758, -1.5705046784850385, 1.6825172981380871])

def mainLoop():
    global ur5
    global robot_data_pub

    rospy.init_node('regraspExp', disable_signals=True)

    # Make this node a service
    dl_service = rospy.ServiceProxy('logData', logData)

    # #Gripper is a C-Model with a TCP connection
    # gripper = robotiq_c_model_control.baseCModel.robotiqBaseCModel()
    # gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # #We connect to the address received as an argument
    # gripper.client.connectToDevice(device)

    # Initialize ur5 interface
    ur5 = UR5Interface()

    ur5.goto_home_pose()

    # Publish robot data constantly
    #robot_data_pub = rospy.Publisher('RobotDataPacket', RobotDataPacket, queue_size=1)

    # Run data logger process
    os.system("rosrun regrasp_exp exp_data_log.py &")
    time.sleep(4)

    # Start logging data
    #resp = dl_service('Start')

    ur5.goto_joint_target(PREGRASP_POSE)

    #resp = dl_service('Stop')

    #while not rospy.is_shutdown():
        # publish robot data
        #robot_data_msg = RobotDataPacket()
        #robot_data_msg.ee_pose = ur5.get_pose()
        #robot_data_pub.publish(robot_data_msg)


        # command trajectories


if __name__ == '__main__':
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
