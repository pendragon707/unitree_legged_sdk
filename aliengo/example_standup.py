#!/usr/bin/python

import sys
import time

sys.path.append('../lib/python/amd64')
import robot_interface_aliengo as sdk

TARGET_PORT = 8007
LOCAL_PORT = 8082
TARGET_IP = "192.168.123.10"   # target IP address

LOW_CMD_LENGTH = 610
LOW_STATE_LENGTH = 771

ALIENGO_LOW_WIRED_DEFAULTS = (LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH, -1) 

LOWLEVEL  = 0xff

def jointLinearInterpolation(initPos, targetPos, rate):

    rate = min(max(rate, 0.0), 1.0)

    p = initPos*(1-rate) + targetPos*rate
    return p

def setup_motor(cmd, num_motor, q, dq, Kp, Kd, tau):
    cmd.motorCmd[num_motor].q = q
    cmd.motorCmd[num_motor].dq = dq
    cmd.motorCmd[num_motor].Kp = Kp
    cmd.motorCmd[num_motor].Kd = Kd
    cmd.motorCmd[num_motor].tau = tau

def main():
    motor_names = {'FR_0', 'FR_1', 'FR_2',
                   'FL_0', 'FL_1', 'FL_2', 
                   'RR_0', 'RR_1', 'RR_2', 
                   'RL_0', 'RL_1', 'RL_2' }
    
    start_q = [-0.15, 1.18, -2.8] * 4
    end_q = [0.05,  0.8, -1.4] * 4
    # end_q = [0.0, 0.3, -1, 0.0, 1, -1] * 2
    qInit = [0] * 12  # массив для начальных состояний моторов
    qDes = [0] * 12
    Kp = [90] * 12
    Kd = [1] * 12

    rate1_count = 0  
    rate2_count = 0  

    udp = sdk.UDP(**ALIENGO_LOW_WIRED_DEFAULTS)

    cmd = sdk.LowCmd()
    udp.InitCmdData(cmd)
    cmd.levelFlag = LOWLEVEL

    state = sdk.LowState()

    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime += 1

        udp.Recv()
        udp.GetRecv(state)

        if( motiontime >= 0):            
            if( motiontime >= 0 and motiontime < 10):
                # Опеределяем начальное положение моторов
                for num, value in enumerate(motor_names):
                    qInit[num] = state.motorState[value].q

            if( motiontime >= 10 and motiontime < 400):
                # Переводим в стартовое положение start_q
                rate1 = min(rate1_count /  200, 1)                     

                for num, value in enumerate(motor_names):
                    qDes[num] = jointLinearInterpolation(qInit[num], start_q[num], rate1)

                rate1_count += 1

            if( motiontime >= 400 ):
                # От стартового положения start_q переходим к end_q
                rate2 = min(rate2_count /  1000, 1)   

                for num, value in enumerate(motor_names):
                    qDes[num] = jointLinearInterpolation(start_q[num], end_q[num], rate2)

                rate2_count += 1                  

            for i in range( 0, len(motor_names) ):
                setup_motor(cmd, i, qDes[i], 0, Kp[i], Kd[i], 0.0)

        udp.SetSend(cmd)
        udp.Send()                

if __name__ == '__main__':
    main()