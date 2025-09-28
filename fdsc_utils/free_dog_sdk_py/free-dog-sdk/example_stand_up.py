from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.lowState import lowState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, LOW_WIFI_DEFAULTS, LOW_WIRED_DEFAULTS
from ucl.enums import GaitType, SpeedLevel, MotorModeLow
from ucl.complex import motorCmd, motorCmdArray
import time
import sys
import math
import numpy as np
from pprint import pprint

# Joint position mappings
d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
     'FL_0':3, 'FL_1':4, 'FL_2':5,
     'RR_0':6, 'RR_1':7, 'RR_2':8,
     'RL_0':9, 'RL_1':10, 'RL_2':11 }

# Define joint positions for standing up and lying down
stand_up_joint_pos = [0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763]
stand_down_joint_pos = [0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375]

print(f'Running lib version: {lib_version()}')
conn = unitreeConnection(LOW_WIFI_DEFAULTS)
conn.startRecv()

lcmd = lowCmd()
lstate = lowState()
mCmdArr = motorCmdArray()

# Send empty command to tell the dog the receive port and initialize the connection
cmd_bytes = lcmd.buildCmd(debug=False)
conn.send(cmd_bytes)

dt = 0.002
running_time = 0.0
motiontime = 0

while True:
    time.sleep(dt)
    motiontime += 1
    running_time += dt

    # Get data
    data = conn.getData()
    for paket in data:
        lstate.parseData(paket)

    if running_time < 3.0:
        # Stand up in first 3 seconds
        phase = math.tanh(running_time / 1.2)
        for i in range(12):
            target_pos = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
            kp = phase * 50.0 + (1 - phase) * 20.0
            kd = 3.5

            # Map index to motor name
            motor_names = ['FR_0', 'FR_1', 'FR_2', 'FL_0', 'FL_1', 'FL_2',
                          'RR_0', 'RR_1', 'RR_2', 'RL_0', 'RL_1', 'RL_2']
            motor_name = motor_names[i]

            mCmdArr.setMotorCmd(motor_name, motorCmd(mode=MotorModeLow.Servo,
                                                     q=target_pos,
                                                     dq=0.0,
                                                     Kp=kp,
                                                     Kd=kd,
                                                     tau=0.0))
        lcmd.motorCmd = mCmdArr
    else:
        # Maintain standing position
        for i in range(12):
            target_pos = stand_up_joint_pos[i]
            kp = 50.0
            kd = 3.5

            motor_names = ['FR_0', 'FR_1', 'FR_2', 'FL_0', 'FL_1', 'FL_2',
                          'RR_0', 'RR_1', 'RR_2', 'RL_0', 'RL_1', 'RL_2']
            motor_name = motor_names[i]

            mCmdArr.setMotorCmd(motor_name, motorCmd(mode=MotorModeLow.Servo,
                                                     q=target_pos,
                                                     dq=0.0,
                                                     Kp=kp,
                                                     Kd=kd,
                                                     tau=0.0))
        lcmd.motorCmd = mCmdArr

    # Send command
    cmd_bytes = lcmd.buildCmd(debug=False)
    conn.send(cmd_bytes)

    if motiontime % 500 == 0:
        print(f'Running time: {running_time:.2f} Phase: {phase:.3f}')