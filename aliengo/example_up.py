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

def main():
    motor_names = {0:'FR_0', 1:'FR_1', 2:'FR_2',
                   3:'FL_0', 4:'FL_1', 5:'FL_2', 
                   6:'RR_0', 7:'RR_1', 8:'RR_2', 
                   9:'RL_0', 10:'RL_1', 11:'RL_2' }
    
    safe = sdk.Safety(sdk.LeggedType.Aliengo)
    
    start_q = [0.0, 1.18, -2.8] * 4
    mid_q = [0.0, 1.18, -2.8]*2 + [0.0, 2.5, -2.8]*2
    end_q = [0.0, 0.3, -1] * 4
    Kp = [90] * 12
    Kd = [1] * 12

    qInit = [0] * 12    
        
    udp = sdk.UDP(*ALIENGO_LOW_WIRED_DEFAULTS)    

    cmd = sdk.LowCmd()
    udp.InitCmdData(cmd)
    cmd.levelFlag = LOWLEVEL

    state = sdk.LowState()

    motiontime = 0
    count1 = 0
    count2 = 0
    count3 = 0
    count4 = 0
    while True:
        time.sleep(0.002)
        motiontime += 1

        udp.Recv()
        udp.GetRecv(state)

        # if motiontime % 100 == 0:
        #     for num, name in motor_names.items():                
        #         print( name, " q ", state.motorState[ num ].q )
        #         print( name, " tau ", state.motorState[ num ].tauEst )
        #         print()

        if( motiontime >= 0 and motiontime < 10):

            for num, value in enumerate(motor_names.items()):
                qInit[num] = state.motorState[num].q

        if( motiontime >= 10 and motiontime < 20):
            rate = min(count1 /  20, 1)

            for num, value in enumerate(motor_names.items()):
                cmd.motorCmd[num].q = jointLinearInterpolation(qInit[num], start_q[num], rate)
                cmd.motorCmd[num].dq = 0
                cmd.motorCmd[num].Kp = Kp[num]
                cmd.motorCmd[num].Kd = Kd[num]
                cmd.motorCmd[num].tau = 0.0

            count1 += 1

        if( motiontime >= 20 and motiontime < 400):
            rate = min(count2 /  380, 1)

            for num, value in enumerate(motor_names.items()):
                cmd.motorCmd[num].q = jointLinearInterpolation(start_q[num], mid_q[num], rate)
                cmd.motorCmd[num].dq = 0
                cmd.motorCmd[num].Kp = Kp[num]
                cmd.motorCmd[num].Kd = Kd[num]
                cmd.motorCmd[num].tau = 0.0

            count2 += 1

        if( motiontime >= 400): 
            alpha = min(count3 /  1000, 1)

            for num, name in motor_names.items(): 
                cmd.motorCmd[num].q = jointLinearInterpolation(mid_q[num], end_q[num], alpha)
                cmd.motorCmd[num].dq = 0
                cmd.motorCmd[num].Kp = Kp[num]
                cmd.motorCmd[num].Kd = Kd[num]
                cmd.motorCmd[num].tau = 0.0

            count3 += 1

        if( motiontime >= 1400): 
            alpha = min(count4 /  1000, 1)

            for num, name in motor_names.items(): 
                cmd.motorCmd[num].q = jointLinearInterpolation(end_q[num], start_q[num], alpha)
                cmd.motorCmd[num].dq = 0
                cmd.motorCmd[num].Kp = Kp[num]
                cmd.motorCmd[num].Kd = Kd[num]
                cmd.motorCmd[num].tau = 0.0

            count4 += 1

        if(motiontime > 10):
            safe.PowerProtect(cmd, state, 1)

        udp.SetSend(cmd)
        udp.Send()



if __name__ == '__main__':
    main()