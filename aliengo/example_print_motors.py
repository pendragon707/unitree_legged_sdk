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

def main():
    motor_names = {0:'FR_0', 1:'FR_1', 2:'FR_2',
                   3:'FL_0', 4:'FL_1', 5:'FL_2', 
                   6:'RR_0', 7:'RR_1', 8:'RR_2', 
                   9:'RL_0', 10:'RL_1', 11:'RL_2' }
        
    udp = sdk.UDP(*ALIENGO_LOW_WIRED_DEFAULTS)    

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

        if motiontime % 100 == 0:
            for num, name in motor_names.items():                
                print( name, " q ", state.motorState[ num ].q )
                print( name, " tau ", state.motorState[ num ].tauEst )
                print()

        udp.SetSend(cmd)
        udp.Send()



if __name__ == '__main__':
    main()