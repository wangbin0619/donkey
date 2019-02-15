#!/usr/bin/env python3

import time
import datetime
import ev3
import struct

now = datetime.datetime.now().strftime('%H:%M:%S.%f')
print ('{} Start >> '.format(now))

my_ev3 = ev3.EV3(protocol=ev3.USB, host='00:16:53:44:23:17')
my_ev3.verbosity = 1

def remote_motor_status(port: int) -> None:

    ops = b''.join([
        ev3.opInput_Device,
        ev3.READY_SI,
        ev3.LCX(0),                   # LAYER
        ev3.port_motor_input(port), # NO
        ev3.LCX(7),                   # TYPE
        ev3.LCX(0),                   # MODE
        ev3.LCX(1),                   # VALUES
        ev3.GVX(0),                   # VALUE1
    ])
    reply = my_ev3.send_direct_cmd(ops, global_mem=4)
    (pos_a,) = struct.unpack('<f', reply[5:])
#    pos_a = struct.unpack('<f', reply[5:])
    print("Get Remote MOTOR in PORT with position {}".format(pos_a))

def remote_motor_stop(port: int) -> None:
    global my_ev3
    ops = b''.join([
        ev3.opOutput_Stop,
        ev3.LCX(0),                       # LAYER
        ev3.LCX(port),                    # NOS
        ev3.LCX(0)                        # BRAKE
    ])
    my_ev3.send_direct_cmd(ops)
    print("Stop Remote MOTOR in PORT", port)

def remote_sensor_mode(port: int) -> None:
    global my_ev3

    ops = b''.join([
        ev3.opInput_Device,
        ev3.GET_TYPEMODE,
        ev3.LCX(0),                   # LAYER
        ev3.LCX(port),                # NO
        ev3.GVX(0),                   # TYPE
        ev3.GVX(1)                    # MODE
    ])
    reply = my_ev3.send_direct_cmd(ops, global_mem=2)
    (type, mode) = struct.unpack('BB', reply[5:7])
    print("Sensor Port: {} type: {}, mode: {}".format(port, type, mode))

def remote_motor_star(speed: int, port: int) -> None:
    global my_ev3

    ops = b''.join([
        ev3.opOutput_Speed,
        ev3.LCX(0),                       # LAYER
        ev3.LCX(port),              # NOS
        ev3.LCX(speed),             # SPEED
        ev3.opOutput_Start,
        ev3.LCX(0),                       # LAYER
        ev3.LCX(ev3.PORT_B)  # NOS
    ])
    my_ev3.send_direct_cmd(ops)
    print("Star Remote MOTOR in PORT",port)

def remote_gyro_input(port: int) -> int:
    global my_ev3
    ops = b''.join([
        ev3.opInput_Device,
        ev3.READY_SI,
        ev3.LCX(0),          # LAYER
        ev3.LCX(port),          # NO
        ev3.LCX(32),         # TYPE - EV3-IR
        ev3.LCX(0),          # MODE - Proximity
        ev3.LCX(1),          # VALUES
        ev3.GVX(0)           # VALUE1
    ])
    reply = my_ev3.send_direct_cmd(ops, global_mem=4)
    input = struct.unpack('<f', reply[5:9])[0] 
    print ("Remote GYRO input: {} in PORT: {}".format(input, port))
    return input

#remote_motor_status(ev3.PORT_B)

#remote_motor_star(50, ev3.PORT_B)

remote_sensor_mode(0)

'''
for i in range (1):
    time.sleep(0.1)
#    print (i,end=' ')
    remote_gyro_input(0)
'''
#remote_motor_stop(ev3.PORT_B)
