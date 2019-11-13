#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
import os, sys
import time

from tae_psoc.msg import SensorPacket
from tae_psoc.msg import cmdToPsoc
from ur5_interface import UR5Interface

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3

currState = IDLE
CMD_in = NO_CMD


def sensor_data_cb(data):
    global last_ft
    last_ft = np.array(data.sensorFT)

def mainLoop():
    global currState
    global CMD_in   
    global ftsensor_pub

    rospy.init_node('regraspExp')

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

    time.sleep(1)
    print("publishing start cmd")
    cmd_msg = cmdToPsoc(START_CMD)
    ftsensor_pub.publish(cmd_msg)
    time.sleep(50)
    cmd_msg = cmdToPsoc(IDLE_CMD)
    print("publishing idle cmd")
    ftsensor_pub.publish(cmd_msg)


    #We loop
    while not rospy.is_shutdown():
        pass
                
if __name__ == '__main__':
    global ftsensor_pub
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        cmd_msg = cmdToPsoc()
        cmd_msg.cmdInput = IDLE_CMD
        ftsensor_pub.publish(cmd_msg)
        raise

