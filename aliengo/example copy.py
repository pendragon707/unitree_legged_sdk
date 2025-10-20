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

def interpolate(start, end, step, total_steps):
    if step >= total_steps:
        return end
    
    alpha = step / total_steps

    return start * (1 - alpha) + end * alpha

def main():
    motor_names = {0:'FR_0', 1:'FR_1', 2:'FR_2',
                   3:'FL_0', 4:'FL_1', 5:'FL_2', 
                   6:'RR_0', 7:'RR_1', 8:'RR_2', 
                   9:'RL_0', 10:'RL_1', 11:'RL_2' }
    
    safe = sdk.Safety(sdk.LeggedType.Aliengo)
    
    start_q = [0.0, 1.18, -2.8]*4
    mid_q = [0.0, 1.18, -2.8]*2 + [0.0, 2.5, -2.8]*2
    end_q = [0.0, 0.3, -1] * 4

    Kp = 90
    Kd = 1
    
    udp = sdk.UDP(*ALIENGO_LOW_WIRED_DEFAULTS)

    cmd = sdk.LowCmd()
    cmd.levelFlag = LOWLEVEL
    udp.InitCmdData(cmd)

    state = sdk.LowState()

    motiontime = 0
    while True:  
        time.sleep(0.002)  
        motiontime += 1 

        udp.Recv()
        udp.GetRecv(state)

        if (motiontime >= 100 and motiontime < 200):
            for num, _name in motor_names.items():                
                cmd.motorCmd[num].q = interpolate(start_q[num], mid_q[num], 100)
                cmd.motorCmd[num].dq = 0.0
                cmd.motorCmd[num].tau = Kp
                cmd.motorCmd[num].Kp = Kd
                cmd.motorCmd[num].Kd = 0.0

        if (motiontime >= 200 and motiontime < 1000):
            for num, _name in motor_names.items():                
                cmd.motorCmd[num].q = interpolate(mid_q[num], end_q[num], 800)
                cmd.motorCmd[num].dq = 0.0
                cmd.motorCmd[num].tau = Kp
                cmd.motorCmd[num].Kp = Kd
                cmd.motorCmd[num].Kd = 0.0          

        if(motiontime > 10):
            safe.PowerProtect(cmd, state, 1)    

        udp.Send()
        udp.SetSend(cmd)



if __name__ == '__main__':
    main()