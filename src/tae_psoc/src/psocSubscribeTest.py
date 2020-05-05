#!/usr/bin/env python

import rospy


import time
import psocScanner as psoc
import struct
import numpy as np
import os, sys
import time



from tae_psoc.msg import SensorPacket
from tae_psoc.msg import cmdToPsoc


IDLE = 0
STREAMING = 1


NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3



currState = IDLE
CMD_in = NO_CMD


def callback(data):
    print(data.sensorCounter)
    #print(data.sensorFT)
    

    

def mainLoop():
    global currState
    global CMD_in   
    global cmd_pub

    # #Gripper is a C-Model with a TCP connection
    # gripper = robotiq_c_model_control.baseCModel.robotiqBaseCModel()
    # gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # #We connect to the address received as an argument
    # gripper.client.connectToDevice(device)

    rospy.init_node('psocSubscribeTest')

    #Each Sensor Reading is Published to topic 'SensorReading'
    cmd_pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=1)
    rospy.Subscriber('SensorPacket',SensorPacket, callback)

    time.sleep(1)
    print("publishing start cmd")
    cmd_msg = cmdToPsoc(START_CMD)
    cmd_pub.publish(cmd_msg)
    time.sleep(50)
    cmd_msg = cmdToPsoc(IDLE_CMD)
    print("publishing idle cmd")
    cmd_pub.publish(cmd_msg)


    #We loop
    while not rospy.is_shutdown():
        pass
                
if __name__ == '__main__':
    global cmd_pub
    try:
        print("Started!")
        mainLoop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        cmd_msg = cmdToPsoc()
        cmd_msg.cmdInput = IDLE_CMD
        cmd_pub.publish(cmd_msg)
        raise









##########################33



# runTime = 15



# SensorExist = 1
# plotShow = 1

# ResultSavingDirectory = '/home/tae/Data/JooyeunShear'
# SavingFileName = 'test_box'



# if SensorExist:
#     import datetime
#     currDateString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")
# #    output_file = ResultSavingDirectory + '\\'+ 'result_' +currDateString + SavingFileName + '.html'

#     ts = psoc.TactileSensor(port="/dev/ttyACM0")
#     ts.ser.flushInput()
    
#     ts.sendChar("i")
#     time.sleep(0.01)
#     ts.sendChar("q")
#     time.sleep(0.01)
    
#     ts.packet_size = ord(ts.ser.read(1))-1
#     ts.num_sensors= (ts.packet_size - 1) / 2
#     ts.unpackFormat = '<'
#     for i in range(0,ts.packet_size):
#         ts.unpackFormat = ts.unpackFormat + 'B'
        
        
    
#     tic = time.time()
    
#     #Start Streaming
#     ts.sendChar("s")
    
#     #%% Get Initial samples for measuring Offset Values
    
#     initialSamplingNum = 15
#     initialData = np.zeros((initialSamplingNum,ts.num_sensors))
    
#     for i in range(0,initialSamplingNum):
#     #    print(ord(ts.ser.read(1)))    
#         while ord(ts.ser.read(1))!= ts.STX:
#             continue
                    
#         #Read rest of the data
#         initialData[i,:] = ts.readRestData()
    
#     #Ignore the first row
#     initialData = initialData[5:None,:]
    
#     initial_offset = np.mean(initialData,axis=0)
    
#     #%% Buffer
#     init_BufferSize = 5000;
#     dataBuffer = np.zeros((init_BufferSize,ts.num_sensors))
#     storingCounter = 0;
        
#     #ts.sendChar("i")
    

# #%%

# tic = time.time()

# stopCMDsent = False


# while time.time() - tic < runTime:
#     if SensorExist:
#         while ord(ts.ser.read(1))!= ts.STX:
#             continue
                    
#         #Read rest of the data
#         dataBuffer[storingCounter,:] = ts.readRestData()
        
        
#         storingCounter += 1
        
#         #if the data overflows
#         if storingCounter > dataBuffer.shape[0]-1:
#             dataBuffer = np.append(dataBuffer, np.zeros((init_BufferSize,ts.num_sensors)), axis=0)
            

# if SensorExist:    
#     dataBuffer = dataBuffer[0:storingCounter-1,:]
#     ts.sendChar("i")

# #r.stopMotion()


# if SensorExist:
#     ts.closePort()
    
#     #r.demo()
    
#     if plotShow:
#         #%% Plot the data
#         import matplotlib.pyplot as plt
#         plt.figure()
#         plt.plot(dataBuffer)
#         plt.ylabel('Digital Count')
#         plt.xlabel('sample')
#         plt.grid()
#         plt.show()
    
#     #%% Save Output
#     import datetime
#     currDateString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")
#     output_file = ResultSavingDirectory + '/'+ 'result_' +currDateString + SavingFileName + '.csv'
    
#     np.savetxt(output_file, dataBuffer, delimiter=",")

