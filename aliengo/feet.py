#!/usr/bin/python

import sys
import time
import math
#import numpy as np


sys.path.append('../lib/python/arm64')

print(sys.path)
import robot_interface as sdk

# low cmd
TARGET_PORT = 8007
LOCAL_PORT = 8082
TARGET_IP = "192.168.123.10"   # target IP address

LOW_CMD_LENGTH = 610
LOW_STATE_LENGTH = 771

def jointLinearInterpolation(initPos, targetPos, rate):

    #rate = np.fmin(np.fmax(rate, 0.0), 1.0)
    if rate > 1.0:
        rate = 1.0
    elif rate < 0.0:
        rate = 0.0

    p = initPos*(1-rate) + targetPos*rate
    return p

def is_on_feet(state) -> bool:
    return False

def setup_motor(cmd, num_motor, q, dq, Kp, Kd, tau):
    cmd.motorCmd[num_motor].q = q
    cmd.motorCmd[num_motor].dq = dq
    cmd.motorCmd[num_motor].Kp = Kp
    cmd.motorCmd[num_motor].Kd = Kd
    cmd.motorCmd[num_motor].tau = tau

def setup_feet(cmd, lists_motors):
    assert all(len(lists_motors[0]) == len(l) for l in lists_motors[1:]), "not valid motor data"

    for motor in lists_motors:
        setup_motor(cmd, motor[0], motor[1], motor[2], motor[3], motor[4], motor[5] )

if __name__ == '__main__':

    d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
         'FL_0':3, 'FL_1':4, 'FL_2':5, 
         'RR_0':6, 'RR_1':7, 'RR_2':8, 
         'RL_0':9, 'RL_1':10, 'RL_2':11 }
    PosStopF  = math.pow(10,9)
    VelStopF  = 16000.0
    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff
    sin_mid_q = [0.0, 1.2, -2.0]*4
    dt = 0.002

    qInit = [0]*12
    qDes = [0]*12
    state = [0]*12
    # freq_get_state = 12

    sin_count = 0
    rate_count = 0
    Kp = [0]*12
    Kd = [0]*12

    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH, -1)
    #udp = sdk.UDP(8082, "192.168.123.10", 8007, 610, 771)
    safe = sdk.Safety(sdk.LeggedType.Aliengo)
    
    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)
    cmd.levelFlag = LOWLEVEL

    Tpi = 0
    motiontime = 0
    while True:

        time.sleep(0.002)
        motiontime += 1

        # print(motiontime)
        # print(state.imu.rpy[0])
        
        udp.Recv()
        udp.GetRecv(state)
        
        if( motiontime >= 0):

            # first, get record initial position
            if( motiontime >= 0 and motiontime < 10):
                for num, value in enumerate(d.values()):
                    qInit[num] = state.motorState[value].q

            # second, move to the origin point of a sine movement with Kp Kd
            if( motiontime >= 10 and motiontime < 400):
                rate_count += 1
                rate = rate_count/200.0                       # needs count to 200
                # Kp = [5, 5, 5]
                # Kd = [1, 1, 1]
                Kp = [20]*12
                Kd = [2]*12
                
                if( motiontime >= 0 and motiontime < 10):
                    for num, value in enumerate(d.values()):
                        qDes[num] = jointLinearInterpolation(qInit[num], sin_mid_q[num], rate)

            # last, do sine wave
            freq_Hz = 1
            # freq_Hz = 5
            freq_rad = freq_Hz * 2* math.pi
            t = dt*sin_count
            if( motiontime >= 400):
                sin_count += 1
                # sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0)
                # sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0)
                sin_joint1 = 0.6 * math.sin(t*freq_rad)
                sin_joint2 = -0.9 * math.sin(t*freq_rad)

                for i in range( 0, len(d), 3 ):
                    qDes[i] = sin_mid_q[i]
                    qDes[i + 1] = sin_mid_q[i + 1] + sin_joint1
                    qDes[i + 2] = sin_mid_q[i + 2] + sin_joint2
            
            for i in range( 0, len(d), 3 ):
                cmd.motorCmd[list(d.values())[i]].q = qDes[i]
                cmd.motorCmd[list(d.values())[i]].dq = 0
                cmd.motorCmd[list(d.values())[i]].Kp = Kp[i]
                cmd.motorCmd[list(d.values())[i]].Kd = Kd[i]
                cmd.motorCmd[list(d.values())[i]].tau = -1.6

                cmd.motorCmd[list(d.values())[i+1]].q = qDes[i + 1]
                cmd.motorCmd[list(d.values())[i+1]].dq = 0
                cmd.motorCmd[list(d.values())[i+1]].Kp = Kp[i + 1]
                cmd.motorCmd[list(d.values())[i+1]].Kd = Kd[i + 1]
                cmd.motorCmd[list(d.values())[i+1]].tau = 0.0

                cmd.motorCmd[list(d.values())[i+2]].q =  qDes[i + 2]
                cmd.motorCmd[list(d.values())[i+2]].dq = 0
                cmd.motorCmd[list(d.values())[i+2]].Kp = Kp[i + 2]
                cmd.motorCmd[list(d.values())[i+2]].Kd = Kd[i + 2]
                cmd.motorCmd[list(d.values())[i+2]].tau = 0.0

            # for i in range( 0, 4 ):
            #     setup_feet(cmd, list(d.values())[i:i+2], [0]*3, Kp[i:i+2], Kd[i:i+2], [-1.6, 0.0, 0.0] )

            for num, value in enumerate(d.values()):
                state[num] = state.motorState[value].q

            if is_on_feet(state):
                break

        if(motiontime > 10):
            safe.PowerProtect(cmd, state, 1)


        udp.SetSend(cmd)
        udp.Send()
