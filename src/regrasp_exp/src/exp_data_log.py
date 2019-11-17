#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
import os, sys
import subprocess
import time

from tae_psoc.msg import SensorPacket
from tae_psoc.msg import cmdToPsoc
from regrasp_exp.srv import logData, logDataResponse
from sensor_msgs.msg import JointState
import tf2_ros

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3

IDLE_LOG = 0
RUN_LOG = 1
STOP_LOG = 2

# global variables
data_log_status = IDLE_LOG


def sensor_data_cb(data):
    global curr_ft_data
    curr_ft_data = np.array(data.sensorFT)

def robot_data_cb(data):
    global curr_robot_data
    curr_robot_data = np.array(data.position)

def write_frame_to_file(fd):
    global curr_ft_data
    global tfBuffer
    robot_pose = get_7dof_from_msg(tfBuffer.lookup_transform('tool0', 'base_link', rospy.Time(0)))
    data = np.hstack((curr_ft_data, curr_robot_data))
    fd.write(','.join([`num` for num in data]) + '\n')
    fd.flush()


def get_7dof_from_msg(msg):
    res = np.zeros(7)
    res[0] = msg.transform.translation.x
    res[1] = msg.transform.translation.y
    res[2] = msg.transform.translation.z
    res[3] = msg.transform.rotation.x
    res[4] = msg.transform.rotation.y
    res[5] = msg.transform.rotation.z
    res[6] = msg.transform.rotation.w
    return res


def service_request(req_arg):
    global data_log_status 
    if (req_arg.cmd == "Start"):
        data_log_status = RUN_LOG
    elif (req_arg.cmd == "Pause"):
        data_log_status = IDLE_LOG
    elif (req_arg.cmd == "Stop"):
        data_log_status = STOP_LOG
    # return resopnse of 1 for success
    return logDataResponse('Success')


def mainLoop():
    global ftsensor_pub
    global tfBuffer
    global curr_ft_data
    global curr_robot_data
    global psoc_proc

    curr_ft_data = np.zeros(6)
    curr_robot_data = np.zeros(6)

    rospy.init_node('regraspData', disable_signals=True)

    os.system('sudo chmod 777 /dev/ttyACM0')
    #psoc_proc = subprocess.Popen(['exec rosrun tae_psoc psocPubSub.py test'], stdout=subprocess.PIPE, shell=True)
    #time.sleep(4)

    rate = rospy.Rate(500)

    # Make this node a service
    s = rospy.Service('logData', logData, service_request)

    #Each Sensor Reading is Published to topic 'SensorReading'
    ftsensor_pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=1)
    rospy.Subscriber('SensorPacket', SensorPacket, sensor_data_cb)
    rospy.Subscriber('joint_states', JointState, robot_data_cb)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    time.sleep(2)
    cmd_msg = cmdToPsoc(IDLE_CMD)
    ftsensor_pub.publish(cmd_msg)
    time.sleep(2)
    cmd_msg = cmdToPsoc(START_CMD)
    ftsensor_pub.publish(cmd_msg)

    data_filepath = os.path.dirname(os.path.abspath(__file__)) + "/data/"
    counter = 0

    time.sleep(10)

    #We loop
    fileN = 0
    while os.path.exists(data_filepath + 'exp%d.txt' % fileN):
        fileN += 1
    #with open(data_filepath + 'exp%d.txt' % fileN, 'w', buffering=1) as redf:
    redf = open(data_filepath + 'exp%d.txt' % fileN, 'w', buffering=1)
    while not rospy.is_shutdown():
        if (data_log_status == RUN_LOG):
            write_frame_to_file(redf)
        if (data_log_status == STOP_LOG):
            cmd_msg = cmdToPsoc(IDLE_CMD)
            ftsensor_pub.publish(cmd_msg)
            redf.close()
            psoc_proc.kill()
            cmd_msg = cmdToPsoc(IDLE_CMD)
            ftsensor_pub.publish(cmd_msg)
            print("Service killed")
            break
        rate.sleep()
                
if __name__ == '__main__':
    global ftsensor_pub
    global psoc_proc
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        cmd_msg = cmdToPsoc(IDLE_CMD)
        ftsensor_pub.publish(cmd_msg)
        print("Keyboard!")
        raise

