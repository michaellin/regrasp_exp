#!/usr/bin/env python

import rospy


import time
import psocScanner as psoc
import struct
import numpy as np
import os, sys
  

from scipy.io import loadmat
from scipy.io import savemat



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
    global CMD_in    
    CMD_in = data.cmdInput


def mainLoop(savingFileName):
    global currState
    global CMD_in   

    # #Gripper is a C-Model with a TCP connection
    # gripper = robotiq_c_model_control.baseCModel.robotiqBaseCModel()
    # gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # #We connect to the address received as an argument
    # gripper.client.connectToDevice(device)

    rospy.init_node('psocPubSub')

    #Each Sensor Reading is Published to topic 'SensorReading'
    pub = rospy.Publisher('SensorPacket', SensorPacket, queue_size=1)
    rospy.Subscriber('cmdToPsoc', cmdToPsoc, callback)

    msg = SensorPacket()

    # #The Gripper command is received from the topic named 'CModelRobotOutput'
    # rospy.Subscriber('CModelRobotOutput', outputMsg.CModel_robot_output, gripper.refreshCommand)    
    
    counter = 0

    # rospy.spin();

    ResultSavingDirectory = '/home/bdml/Tae_Data'
    SavingFileName = savingFileName  #'test_box'

    SensorExist = 1
    plotShow = 1

    #Read Calibration File
    MatFile = loadmat('/home/bdml/Tae_Data/Michael_CalMatrix/Michael_CalMatrix_2_0')
    # CalMat_A = MatFile['A_trainsets']
    CalMat_A = MatFile['A_trainsets']
    fitOrder = MatFile['fittingOrder']

    

    #We loop
    while not rospy.is_shutdown():
        if currState == IDLE and CMD_in == START_CMD:
            CMD_in = NO_CMD
            currState = STREAMING

            # sensor_1_FT_history_second = np.zeros((init_BufferSize, 6), dtype='f')
            sensor_current_FT = np.zeros((1, 6), dtype='f')

                 

            if SensorExist:
                import datetime              
                currDateTimeString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")
            #    output_file = ResultSavingDirectory + '\\'+ 'result_' +currDateString + SavingFileName + '.html'

                ts = psoc.TactileSensor(port="/dev/ttyACM0")
                ts.ser.flushInput()
                
                ts.sendChar("i")
                time.sleep(0.01)
                ts.sendChar("q")
                time.sleep(0.01)
                
                ts.packet_size = ord(ts.ser.read(1))-1
                ts.num_sensors= (ts.packet_size - 1) / 2
                ts.unpackFormat = '<'
                for i in range(0,ts.packet_size):
                    ts.unpackFormat = ts.unpackFormat + 'B'
                    
                    
                
                tic = time.time()
                
                #Start Streaming
                ts.sendChar("s")
                
                #%% Get Initial samples for measuring Offset Values
                
                initialSamplingNum = 15
                initialData = np.zeros((initialSamplingNum,ts.num_sensors))
                
                for i in range(0,initialSamplingNum):
                #    print(ord(ts.ser.read(1)))    
                    while ord(ts.ser.read(1))!= ts.STX:
                        continue
                                
                    #Read rest of the data
                    initialData[i,:] = ts.readRestData()
                
                #Ignore the first row
                initialData = initialData[5:None,:]
                
                initial_offset = np.mean(initialData,axis=0)
                
                #%% Buffer
                init_BufferSize = 5000;
                dataBuffer = np.zeros((init_BufferSize,ts.num_sensors))
                FT_Buffer = np.zeros((init_BufferSize,6))
                storingCounter = 0;
                    
                #ts.sendChar("i")
                


                # Loop for getting sensor readings

                while not CMD_in == IDLE_CMD:

                    while ord(ts.ser.read(1))!= ts.STX:
                        continue

                                
                    #Read rest of the data
                    dataBuffer[storingCounter,:] = ts.readRestData()

                    #Obtain Calibrated REading
                    # Multiply the calibration matrix accordingly
                    thisRead_withoutOffset = dataBuffer[storingCounter,:] - initial_offset
                    
                    if fitOrder > 2: # include constant coefficient
                        toCalibrate = np.append(1, thisRead_withoutOffset, np.square(thisRead_withoutOffset))
                    else: # without coefficient
                        toCalibrate = np.append(thisRead_withoutOffset, np.square(thisRead_withoutOffset))

                    sensor_current_FT[0,:] = np.matmul(CalMat_A, toCalibrate)
                    FT_Buffer[storingCounter,:] = sensor_current_FT


                    #Get and publish the Each Sensor Read

                    msg.sensorFT = sensor_current_FT[0,:] #sensor_current_FT[0,:] #[0.0 ,1.1, 2.2, 3.3, 4.4, 5.5, 6.6]
                    msg.sensorCounter = storingCounter;
                    
                    pub.publish(msg)
                    
                    storingCounter += 1
                    
                    #if the data overflows
                    if storingCounter > dataBuffer.shape[0]-1:
                        dataBuffer = np.append(dataBuffer, np.zeros((init_BufferSize,ts.num_sensors)), axis=0)
                        FT_Buffer = np.append(FT_Buffer, np.zeros((init_BufferSize,6)), axis=0)
            

                
                dataBuffer = dataBuffer[0:storingCounter-1,:]
                FT_Buffer = FT_Buffer[0:storingCounter-1,:]
                ts.sendChar("i")


                #r.stopMotion()
                
                ts.closePort()
                
                #r.demo()
                
                if plotShow:
                    #%% Plot the data
                    import matplotlib.pyplot as plt
                    plt.figure()
                    plt.plot(FT_Buffer)
                    plt.ylabel('Digital Count')
                    plt.xlabel('sample')
                    plt.grid()
                    plt.show(block=False)

                    plt.pause(3)
                    plt.close()
                
                #%% Save Output
                import datetime
                currDateOnlyString = datetime.datetime.now().strftime("%y%m%d")
                directory = ResultSavingDirectory +'/' + currDateOnlyString
                if not os.path.exists(directory):
                	os.makedirs(directory)

                
                output_file = directory + '/'+ 'result_' +currDateTimeString + SavingFileName + '.csv'
                	
                # # currDateString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")
                # output_file = ResultSavingDirectory + '/'+ 'result_' +currDateString + SavingFileName + '.csv'
                

                # in case you want .mat file
                output_file = directory + '/'+ 'result_' +currDateTimeString + SavingFileName + '.mat'
                
                
                # # Save as .mat file
                # savemat(output_file, 
                #     {'sensor_1_data_history_first':sensor_1_data_history_first,
                #      'fftStorage_sns_1_NS':fftStorage_sns_1_NS,
                #       'fftStorage_sns_1_WE':fftStorage_sns_1_WE,
                #       'sensor_1_data_history_second' : sensor_1_data_history_second,
                #       'sensor_1_FT_history_second' : sensor_1_FT_history_second,
                #       'sensor_2_data_history_first':sensor_2_data_history_first,
                #       'fftStorage_sns_2_NS':fftStorage_sns_2_NS,
                #       'fftStorage_sns_2_WE':fftStorage_sns_2_WE,
                #       'sensor_2_data_history_second' : sensor_2_data_history_second,
                #       'sensor_2_FT_history_second' : sensor_2_FT_history_second,
                #       'GripperPosCMD_History' : GripperPosCMD_History,
                #       'sensingMode' : sensingMode,
                #       'sensor_1_vorticity_history_second' : sensor_1_vorticity_history_second,
                #       'sensor_2_vorticity_history_second' : sensor_2_vorticity_history_second,
                #       'UR5_Y_pos' : UR5_Y_pos})



                np.savetxt(output_file, dataBuffer, delimiter=",")
                print("file Saved")

                CMD_in = NO_CMD
                currState = IDLE














                
if __name__ == '__main__':
    try:
        print("Started!")
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException: pass









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

