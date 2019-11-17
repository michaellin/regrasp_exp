#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
import os, sys
import time

from sensor_msgs.msg import JointState
import tf2_ros

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3

IDLE_LOG = 0
RUN_LOG = 1
STOP_LOG = 2



def mainLoop():
    rospy.init_node('test')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #rospy.Subscriber('SensorPacket', SensorPacket, sensor_data_cb)
    #rospy.Subscriber('joint_states', JointState, robot_data_cb)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ee_link', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")
            rate.sleep()
            continue
                
        print(trans.transform)
if __name__ == '__main__':
    mainLoop()

