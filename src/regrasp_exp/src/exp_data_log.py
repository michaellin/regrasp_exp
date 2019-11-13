#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
import os, sys
import time

from tae_psoc.msg import SensorPacket
from tae_psoc.msg import cmdToPsoc
from regrasp_exp.srv import logData, logDataResponse
from sensor_msgs.msg import JointState

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3

IDLE_LOG = 0
RUN_LOG = 1
STOP_LOG = 2


def sensor_data_cb(data):
    global curr_ft_data
    curr_ft_data = np.array(data.sensorFT)

def robot_data_cb(data):

def write_frame_to_file(fd):
    global curr_ft_data
    global ur5
    data = np.concatenate(curr_ft_data, ur5.get_pose())
    print(data, file = fd, flush=True)


def service_request(req_arg):
    global data_log_status 
    if (req_arg.cmd == "Start"):
        data_log_status = RUN_LOG
    elif (req_arg.cmd == "Pause"):
        data_log_status = IDLE_LOG
    elif (req_arg.cmd == "Stop"):
        data_log_status = STOP_LOG
    # return resopnse of 1 for success
    return dataLogResponse(1)


def mainLoop():
    global ftsensor_pub
    global ur5
    global data_log_status 

    rospy.init_node('regraspExp')

    # Make this node a service
    s = rospy.Service('logData', logData, service_request)

    # #Gripper is a C-Model with a TCP connection
    # gripper = robotiq_c_model_control.baseCModel.robotiqBaseCModel()
    # gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # #We connect to the address received as an argument
    # gripper.client.connectToDevice(device)


    # Initialize ur5 interface
    ur5 = UR5Interface()

    #Each Sensor Reading is Published to topic 'SensorReading'
    ftsensor_pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=1)
    rospy.Subscriber('SensorPacket', SensorPacket, sensor_data_cb)
    rospy.Subscriber('joint_states', JointState, robot_data_cb)

    time.sleep(1)
    print("publishing start cmd")
    cmd_msg = cmdToPsoc(START_CMD)
    ftsensor_pub.publish(cmd_msg)
    time.sleep(50)
    cmd_msg = cmdToPsoc(IDLE_CMD)
    print("publishing idle cmd")
    ftsensor_pub.publish(cmd_msg)

    data_filepath = os.path.dirname(os.path.abspath(__file__)) + "/data/"


    #We loop
    fileN = 0
    while os.path.exists(data_filepath + 'exp%d.txt' % fileN):
        fileN ++
    with open(data_filepath + 'exp%d.txt' % i, 'w', buffering=1) as redf:
        while not rospy.is_shutdown():
            if (data_log_status == RUN_LOG):
                write_frame_to_file(redf)
            if (data_log_status == STOP_LOG):
                cmd_msg = cmdToPsoc(IDLE_CMD)
                ftsensor_pub.publish(cmd_msg)
                break
                
if __name__ == '__main__':
    global ftsensor_pub
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        cmd_msg = cmdToPsoc(IDLE_CMD)
        ftsensor_pub.publish(cmd_msg)
        raise

