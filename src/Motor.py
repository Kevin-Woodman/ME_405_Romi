## @file Motor.py
# This file contains the driver for the motor of the Gearmotor and Encoder Assembly for Romi/TI-RSLK MAX

from pyb import Pin, Timer

## Implements motor driver
# This class allows control of the motor in the Gearmotor and Encoder Assembly for Romi/TI-RSLK MAX
# The motor drivers uses separate PWM, direction, and sleep inputs.
# @b Example:
# @code
# tm2 = Timer(2, freq=50*100000)
# # Create the motor object
# motor = Motor((tm2, 1, Pin.board.PA15), Pin.board.PH0, Pin.board.PH1)
# # Enable the motor object
# motor.enable()
# # Set the motor object's effort
# motor.set_effort(100)
# # Disable the motor object
# motor.disable()
# @endcode
class Motor:

    ## Initializes the motor object
    # This function initializes a motor object, configuring the provided pins
    # @code
    # tm2 = Timer(2, freq=50*100000)
    # motor = Motor((tm2, 1, Pin.board.PA15), Pin.board.PH0, Pin.board.PH1)
    # @endcode
    # @param PWM A tuple containing a timer, timer channel number, and a PWM pin.
    # @param DIR The direction pin
    # @param nSLP The sleep pin
    def __init__(self, PWM, DIR, nSLP):
        self.PWM_pin = PWM[0].channel(PWM[1], pin=PWM[2], mode=Timer.PWM, pulse_width_percent=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        pass

    ## Sets the present effort
    # Sets the present effort requested from the motor based on an input value between -100 and 100
    # @param effort A float between -100 and 100
    def set_effort(self, effort):
        if effort < 0:
            self.DIR_pin.high()
            self.PWM_pin.pulse_width_percent(-effort)
        else:
            self.DIR_pin.low()
            self.PWM_pin.pulse_width_percent(effort)

    ## Enables the motor driver
    # Enables the motor driver by taking it out of sleep mode into brake mode
    def enable(self):
        self.PWM_pin.pulse_width_percent(0)
        self.nSLP_pin.high()

    ## Disables the motor driver
    # Disables the motor driver by taking it into sleep mode
    def disable(self):
        self.nSLP_pin.low()