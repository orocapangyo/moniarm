import time

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
           self, channel, address, frequency=60, busnum=None, init_delay=0.1
    ):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        #time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise
        self.running = True

    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pwm(pulse)

    def set_pwm_clear(self):
        self.pwm.set_all_pwm(0,0)    

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)     

class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init Steer ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, steering):
        pulse = int(steering)
        print("steer : " + str(steering))
        if steering > 0:
            #Motorhat A
            self.controller.pwm.set_pwm(self.controller.channel,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,4095)
        else:
            self.controller.pwm.set_pwm(self.controller.channel,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,0)

    def shutdown(self):
        self.run(0) #stop vehicle

class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init Throttle ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        pulse = int(throttle)
        if throttle > 0:
            self.controller.pwm.set_pwm(self.controller.channel+3,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,4095)   
        else:
            self.controller.pwm.set_pwm(self.controller.channel+3,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,4095) 

    def shutdown(self):
        self.run(0) #stop vehicle

class PWMThrottleHat:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init Throttle ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        pulse = int(throttle)
        if throttle > 0:   
            # MotorHat B
            self.controller.pwm.set_pwm(self.controller.channel+ 5,0,left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 4,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+ 3,0,4095)

        else:
            # MotorHat B
            self.controller.pwm.set_pwm(self.controller.channel+ 5,0,-left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 4,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+ 3,0,0)

    def shutdown(self):
        self.run(0) #stop vehicle

class PWMThrottle2Wheel:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init ESC 2Wheel")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, throttle, steering):
        left_motor_speed = throttle
        right_motor_speed = throttle

        if steering < 0:
            left_motor_speed *= (1.0 - (-steering/4095)) 
        elif steering > 0:
            right_motor_speed *= (1.0 - (steering/4095))

        left_pulse = int(left_motor_speed)   
        right_pulse = int(right_motor_speed)

        print(
            "left_pulse  : "
            + str(left_pulse)
            + "   / "
            + "right_pulse : "
            + str(right_pulse)
        )

        self.controller.pwm.set_pwm(self.controller.channel+15,0,0)             #BRK for BLDC driver

        if left_motor_speed > 0:
            #rear motor
            #1st L298N, Motorhat B
            self.controller.pwm.set_pwm(self.controller.channel+ 5,0,left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 4,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+ 3,0,4095)
            #2nd L298N, Jetbot
            self.controller.pwm.set_pwm(self.controller.channel+ 8,0,left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 9,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+10,0,0)
        else:
            #front motor
            #1st L298N, Motorhat B
            self.controller.pwm.set_pwm(self.controller.channel+ 5,0,-left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 3,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+ 4,0,4095)       
            #2nd L298N, Jetbot
            self.controller.pwm.set_pwm(self.controller.channel+ 8,0,-left_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 9,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+10,0,4095)           

        if right_motor_speed > 0:
            #rear motor
            #1st L298N, MotorHat A
            self.controller.pwm.set_pwm(self.controller.channel+ 0,0,right_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 1,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+ 2,0,0) 
            #2nd L298N, Jetbot
            self.controller.pwm.set_pwm(self.controller.channel+13,0,right_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+11,0,0) 
            self.controller.pwm.set_pwm(self.controller.channel+12,0,4095)
        else:
            #front motor
            #1st L298N, MotorHat A
            self.controller.pwm.set_pwm(self.controller.channel+ 0,0,-right_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+ 1,0,0) 
            self.controller.pwm.set_pwm(self.controller.channel+ 2,0,4095)
            #2nd L298N, Jetbot
            self.controller.pwm.set_pwm(self.controller.channel+13,0,-right_pulse)
            self.controller.pwm.set_pwm(self.controller.channel+11,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+12,0,0) 
            
            
    def shutdown(self):
        self.run(0) #stop vehicle