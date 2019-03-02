#!/usr/bin/env python3

import os
import sys
import time
import struct
import datetime

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    now = datetime.datetime.now().strftime('%H:%M:%S.%f')
    print(now, end=' ', file=sys.stderr)
    print(*args, **kwargs, file=sys.stderr)

debug_print('----------- Program Start ------------')

import ev3dev2.motor as ev3_master_motor # LargeMotor,OUTPUT_A, OUTPUT_B, OUTPUT_C,OUTPUT_D
import ev3dev2.sensor as ev3_master_sensor_port # INPUT_1,INPUT_2,INPUT_3,INPUT_4
import ev3dev2.sensor.lego as ev3_master_sensor # ColorSensor
import ev3dev2.button as ev3_master_button #Button

DaisyChainEnabled = False
if DaisyChainEnabled:
    import rpyc
    conn = rpyc.classic.connect('ev3devS1')
    ev3_slave_motor = conn.modules['ev3dev2.motor']
    ev3_slave_sensor_port = conn.modules['ev3dev2.sensor'] 
    ev3_slave_sensor = conn.modules['ev3dev2.sensor.lego']

# state constants
ON = True
OFF = False

def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')

def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')

def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)

def main():
    '''The main function of our program'''
    debug_print('----------- Main Start ------------')

    # set the console just how we want it
    reset_console() #重置屏幕
    set_cursor(OFF) #
    set_font('Lat15-Terminus24x12')

    #菊连主机资源
    if DaisyChainEnabled :
        Gyro_slave = ev3_slave_sensor.GyroSensor(ev3_slave_sensor_port.INPUT_1)
        Motor_slave = ev3_slave_motor.MediumMotor(ev3_slave_motor.OUTPUT_B)

    #本地主机资源
    MA = ev3_master_motor.LargeMotor(ev3_master_motor.OUTPUT_A)
    MB = ev3_master_motor.LargeMotor(ev3_master_motor.OUTPUT_B)
    MC = ev3_master_motor.LargeMotor(ev3_master_motor.OUTPUT_C)
    MD = ev3_master_motor.LargeMotor(ev3_master_motor.OUTPUT_D)

    MA.off()  
    MB.off()  
    MC.off()  
    MD.off()  

    if DaisyChainEnabled :
        Motor_slave.off()

    col1 = ev3_master_sensor.ColorSensor(ev3_master_sensor_port.INPUT_1)
    col2 = ev3_master_sensor.ColorSensor(ev3_master_sensor_port.INPUT_2)
    col3 = ev3_master_sensor.ColorSensor(ev3_master_sensor_port.INPUT_3)
    col4 = ev3_master_sensor.ColorSensor(ev3_master_sensor_port.INPUT_4)

    btn = ev3_master_button.Button()

    #巡线主程序
    error=0
    kp=0.6
    speed = 40
    angle = 0
    n1_threshold = 25
    n2_threshold = 25
    n3_threshold = 25
    n4_threshold = 25

    debug_print('----------- Loop Start ------------')
    while not btn.any() :

        if DaisyChainEnabled :
            angle = Gyro_slave.angle
        
        n1=col1.reflected_light_intensity
        n2=col2.reflected_light_intensity
        n3=col3.reflected_light_intensity
        n4=col4.reflected_light_intensity

        if n1 > 60 :
            n1 = 60
        if n2 > 60 :
            n2 = 60

        Turn=n1-n2-error
        l = r = speed

        if n3 < n3_threshold or n4 < n4_threshold :

            if n3 < n3_threshold :
#                if n1 >  n1_threshold :
                    l = -20
                    r = 40
            else :
#                if n2 > n2_threshold :
                    l = 40
                    r = -20
        elif n1 < n1_threshold or n2 < n2_threshold :
            l = 30 + Turn*kp        
            r = 30 - Turn*kp
        else :
            pass

        MA.on(speed = l)  
        MB.on(speed = l) 
        MC.on(speed = r) 
        MD.on(speed = r)

        if DaisyChainEnabled :
            Motor_slave.on(speed = l)

        debug_print('>> Color {:>3d} {:>3d} {:>3d} {:>3d}   Angle {:>6d} >> Left {:>3.1f}  Right {:>3.1f}'.format(n1,n2,n3,n4,angle,l,r))
#        print(n1,n2,n3,n4,angle)
#        print('L {} R {}'.format(l,r))

    MA.off()  
    MB.off()  
    MC.off()  
    MD.off()  

    if DaisyChainEnabled :
        Motor_slave.off()

    debug_print('----------- Main End ------------')


if __name__ == '__main__':
    main()

