"""
actuators.py
Classes to control the motors and servos. These classes 
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk

# package for EV3 Controller
import datetime
import rpyc

flag = False

class EV3_Controller:
    '''
    EV3 motor controller using EV3 brick
    '''
    def __init__(self):
        # Initialise the EV3 module
        global flag
        
        # It is based on 1) Raspberry Pi and EV3 connected by USB. 2) Set to Tethering + Gadget mode. So EV3 get IP address as 192.168.0.1
        self.conn = rpyc.classic.connect('192.168.0.1')

        self.ev3_motor = self.conn.modules['ev3dev2.motor']
        self.steer_pair = self.ev3_motor.MoveSteering(self.ev3_motor.OUTPUT_B, self.ev3_motor.OUTPUT_C)

        #self.ev3_sound = self.conn.modules['ev3dev2.sound']
        #self.sound = self.ev3_sound.Sound()
        
        self.ev3_gyro = self.conn.modules['ev3dev2.sensor.lego']
        self.gyro = self.ev3_gyro.GyroSensor()
        
        self.data_accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.data_gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        
        print ("Initialise the EV3 module")

        self.pre_angle = 0
        self.pre_throttle = 0
        self.steer_pair.on(steering=self.pre_angle, speed=self.pre_throttle)

    def set_pulse(self, pulse):
        try:
            pass
            #print ("Bin's update - set_pulse", tmp++)
            #self.pwm.set_pwm(self.channel, 0, pulse)
        except OSError as err:
            print("Unexpected issue setting PWM (check wires to motor board): {0}".format(err))

    def set_Steering(self, angle):
        try:
            angle = int(angle * 20)
            if angle < 1 and angle > -1 :
                angle = 0

            if angle == self.pre_angle:
                #print("skip since angle keep same: angle {} pre_angle {}".format(angle, self.pre_angle))
                return
            else:
                self.pre_angle = angle

            now = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(now + " Ev3 Steering <<< >>> Angle: {:+3d} Throrrle: {:+3d}".format(angle,self.pre_throttle))
            self.steer_pair.on(steering=angle,speed=self.pre_throttle)

        except OSError as err:
            print("Unexpected issue setting Steering (check wires to motor board): {0}".format(err))

    def set_Throttle(self, throttle):
        try:
            throttle = int(throttle * 60)

            if throttle == self.pre_throttle:
                #print("skip since throttle keep same: throttle {} pre_throttle {}".format(throttle,self.pre_throttle))
                return
            else:
                self.pre_throttle = throttle

            now = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(now + " Ev3 Throttle ^^^ vvv Throttle: {:+3d} Angle: {:+3d} ".format(throttle,self.pre_angle))
            self.steer_pair.on(steering=self.pre_angle,speed=throttle)
            # time.sleep(0.5)

        except OSError as err:
            print("Unexpected issue setting Throttle EV3 (check wires to motor board): {0}".format(err))

    '''
    def start_sound(self):

        # see http://espeak.sourceforge.net/
        # values -a 200 -s 130 SHOULD BE INCLUDED if specifying any other options
        # a = amplitude (200 max, 100 default), s = speed 80-500, default = 175)

        # str_en = "I think you ought to know, I'm feeling very depressed"
        str_zh = "zhun bei chu fa"
        self.sound.speak(str_zh, espeak_opts='-a 200 -s 130 -zh')
    '''

    def get_gyro(self):
        try:
            #self.gyro.angle
            #self.gyro.rate 
            #self.gyro.angle_and_rate
            now = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(now + " Ev3 Gyro - Angle: {:+3d} Rate: {:+3d}".format(self.gyro.angle, self.gyro.rate))
#            print(now + " Ev3 Gyro - Angle_and_rate: ", self.gyro.rate_and_angle)

        except OSError as err:
            print("Unexpected issue getting Gyro (check wires to motor board): {0}".format(err))

        return (self.data_accel['x'], self.data_accel['y'], self.data_accel['z'], 
                self.data_gyro['x'], self.data_gyro['y'], self.data_gyro['z'])

    def run_threaded(self):
        # it is for getting EV3 gyro info instead of instead of throttle/angle control
        return (self.data_accel['x'], self.data_accel['y'], self.data_accel['z'], 
                self.data_gyro['x'], self.data_gyro['y'], self.data_gyro['z'])        

#        return self.get_gyro()

    def run(self, pulse):
        print ("Bin's update - run")
        #self.set_pulse(pulse)

    def update(self):
        pass

class PCA9685:
    ''' 
    PWM motor controler using PCA9685 boards. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            #replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse) 

    def run(self, pulse):
        self.set_pulse(pulse)

class JHat:
    ''' 
    PWM motor controler using Teensy emulating PCA9685. 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        print("Firing up the Hat")
        import Adafruit_PCA9685
        LED0_OFF_L         = 0x08
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            #replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        self.register = LED0_OFF_L+4*channel

        #we install our own write that is more efficient use of interrupts
        self.pwm.set_pwm = self.set_pwm
        
    def set_pulse(self, pulse):
        self.set_pwm(self.channel, 0, pulse) 

    def set_pwm(self, channel, on, off):
        #print("pulse", off)
        """Sets a single PWM channel."""
        self.pwm._device.writeList(self.register, [off & 0xFF, off >> 8])
        
    def run(self, pulse):
        self.set_pulse(pulse)

class JHatReader:
    ''' 
    Read RC controls from teensy 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.register = 0 #i2c read doesn't take an address
        self.steering = 0
        self.throttle = 0
        self.running = True
        #send a reset
        self.pwm._device.writeRaw8(0x06)

    def read_pwm(self):
        '''
        send read requests via i2c bus to Teensy to get
        pwm control values from last RC input  
        '''
        h1 = self.pwm._device.readU8(self.register)
        #first byte of header must be 100, otherwize we might be reading
        #in the wrong byte offset
        while h1 != 100:
            print("skipping to start of header")
            h1 = self.pwm._device.readU8(self.register)
        
        h2 = self.pwm._device.readU8(self.register)
        #h2 ignored now

        val_a = self.pwm._device.readU8(self.register)
        val_b = self.pwm._device.readU8(self.register)
        self.steering = (val_b << 8) + val_a
        
        val_c = self.pwm._device.readU8(self.register)
        val_d = self.pwm._device.readU8(self.register)
        self.throttle = (val_d << 8) + val_c

        #scale the values from -1 to 1
        self.steering = (((float)(self.steering)) - 1500.0) / 500.0  + 0.158
        self.throttle = (((float)(self.throttle)) - 1500.0) / 500.0  + 0.136
        
        #print(self.steering, self.throttle)

    def update(self):
        while(self.running):
            self.read_pwm()
            time.sleep(0.015)
        
    def run_threaded(self):
        return self.steering, self.throttle

    def shutdown(self):
        self.running = False
        time.sleep(0.1)


class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1 
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse


    def run(self, angle):
        '''
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)
        '''
        # wangbin
        self.controller.set_Steering(angle)

    def run_threaded(self, angle):
        '''
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)
        '''
        # wangbin
        self.controller.set_Steering(angle)

    def update(self):
        pass

    def shutdown(self):
        self.run(0) #set steering straight


class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        
        #send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        '''
        if throttle > 0:
            pulse = dk.utils.map_range(throttle,
                                    0, self.MAX_THROTTLE, 
                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.utils.map_range(throttle,
                                    self.MIN_THROTTLE, 0, 
                                    self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)
        '''
        # wangbin
        self.controller.set_Throttle(throttle)

    def update(self):
        pass

    def run_threaded(self, throttle):
        '''
        if throttle > 0:
            pulse = dk.utils.map_range(throttle,
                                    0, self.MAX_THROTTLE, 
                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.utils.map_range(throttle,
                                    self.MIN_THROTTLE, 0, 
                                    self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)
        '''
        # wangbin
        self.controller.set_Throttle(throttle)

    def shutdown(self):
        self.run(0) #stop vehicle



class Adafruit_DCMotor_Hat:
    ''' 
    Adafruit DC Motor Controller 
    Used for each motor on a differential drive car.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit
        
        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60) 
        
        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num
        
        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0
    
        
    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(abs(speed), -1, 1, -255, 255))
        
        if speed > 0:            
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)
            
        self.motor.setSpeed(self.throttle)
        

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)

class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout = 0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout = 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass


class L298N_HBridge_DC_Motor(object):
    '''
    Motor controlled with an L298N hbridge from the gpio pins on Rpi
    '''
    def __init__(self, pin_forward, pin_backward, pwm_pin, freq = 50):
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pwm_pin, freq)
        self.pwm.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        max_duty = 90 #I've read 90 is a good max
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -max_duty, max_duty))
        
        if self.throttle > 0:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.HIGH)
            GPIO.output(self.pin_backward, GPIO.LOW)
        elif self.throttle < 0:
            self.pwm.ChangeDutyCycle(-self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.HIGH)
        else:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.LOW)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class TwoWheelSteeringThrottle(object):

    def run(self, throttle, steering):
        if throttle > 1 or throttle < -1:
            raise ValueError( "throttle must be between 1(forward) and -1(reverse)")
 
        if steering > 1 or steering < -1:
            raise ValueError( "steering must be between 1(right) and -1(left)")

        left_motor_speed = throttle
        right_motor_speed = throttle
 
        if steering < 0:
            left_motor_speed *= (1.0 - (-steering))
        elif steering > 0:
            right_motor_speed *= (1.0 - steering)

        return left_motor_speed, right_motor_speed

    def shutdown(self):
        pass


class Mini_HBridge_DC_Motor_PWM(object):
    '''
    Motor controlled with an mini hbridge from the gpio pins on Rpi
    This can be using the L298N as above, but wired differently with only
    two inputs and no enable line.
    https://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Dtoys-and-games&field-keywords=Mini+Dual+DC+Motor+H-Bridge+Driver
    https://www.aliexpress.com/item/5-pc-2-DC-Motor-Drive-Module-Reversing-PWM-Speed-Dual-H-Bridge-Stepper-Motor-Mini
    '''
    def __init__(self, pin_forward, pin_backward, freq = 50, max_duty = 90):
        '''
        max_duy is from 0 to 100. I've read 90 is a good max.
        '''
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.max_duty = max_duty
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        
        self.pwm_f = GPIO.PWM(self.pin_forward, freq)
        self.pwm_f.start(0)
        self.pwm_b = GPIO.PWM(self.pin_backward, freq)
        self.pwm_b.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed is None:
            return
        
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -self.max_duty, self.max_duty))
        
        if self.throttle > 0:
            self.pwm_f.ChangeDutyCycle(self.throttle)
            self.pwm_b.ChangeDutyCycle(0)
        elif self.throttle < 0:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(-self.throttle)
        else:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm_f.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.pwm_f.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def map_frange(self, x, X_min, X_max, Y_min, Y_max):
    ''' 
    Linear mapping between two ranges of values 
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min)

    return y
    
class RPi_GPIO_Servo(object):
    '''
    Servo controlled from the gpio pins on Rpi
    '''
    def __init__(self, pin, freq = 50, min=5.0, max=7.8):
        import RPi.GPIO as GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pin, freq)
        self.pwm.start(0)
        self.min = min
        self.max = max

    def run(self, pulse):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        #I've read 90 is a good max
        self.throttle = map_frange(pulse, -1.0, 1.0, self.min, self.max)
        #print(pulse, self.throttle)
        self.pwm.ChangeDutyCycle(self.throttle)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class ServoBlaster(object):
    '''
    Servo controlled from the gpio pins on Rpi
    This uses a user space service to generate more efficient PWM via DMA control blocks.
    Check readme and install here:
    https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
    cd PiBits/ServoBlaster/user
    make
    sudo ./servod
    will start the daemon and create the needed device file:
    /dev/servoblaster

    to test this from the command line:
    echo P1-16=120 > /dev/servoblaster

    will send 1200us PWM pulse to physical pin 16 on the pi.

    If you want it to start on boot:
    sudo make install
    '''
    def __init__(self, pin):
        self.pin = pin
        self.servoblaster = open('/dev/servoblaster', 'w')
        self.min = min
        self.max = max

    def set_pulse(self, pulse):
        s = 'P1-%d=%d\n' % (self.pin, pulse)
        self.servoblaster.write(s)
        self.servoblaster.flush()

    def run(self, pulse):
        self.set_pulse(pulse)

    def shutdown(self):
        self.run((self.max + self.min) / 2)
        self.servoblaster.close()

