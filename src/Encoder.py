## @file Encoder.py
# This file contains the driver for the encoder in the Gearmotor and Encoder Assembly for Romi/TI-RSLK MAX

from time import ticks_us, ticks_diff # Use to get dt value in update()
from pyb import Timer
import math

## Implements a quadrature encoder driver.
# This class allows for control of the encoder in the Gearmotor and Encoder Assembly for Romi/TI-RSLK MAX.
# The class uses two timer channel pin to read the state of the quadrature encoder.
# @b Example:
# @code
# # Create the encoder object
# encoder = Encoder(1, Pin.board.PA8, Pin.board.PA9)
# # Update the encoder object
# encoder.update()
# # Get the encoders position (in ticks)
# pos = encoder.get_position()
# # Get the encoders velocity (in ticks/s)
# vel = encoder.get_velocity()
# # Zero the encoder
# encoder.zero()
# @endcode
class Encoder:

    ## Initializes the motor object.
    # This function initializes a motor object, configuring the provided pins and timer.
    # @code
    # encoder = Encoder(1, Pin.board.PA8, Pin.board.PA9)
    # @endcode
    # @param tim The numer of the timer to be configured
    # @param chA_pin The timer's Channel A pin
    # @param chB_pin The timer's Channel B pin

    def __init__(self, tim, chA_pin, chB_pin):
        self.tim = Timer(tim, period = 0xFFFF, prescaler = 0)
        self.tim.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=Timer.ENC_AB)
        self.position = 0 # Total accumulated position of the encoder
        self.prev_count = 0 # Counter value from the most recent update
        self.delta = 0 # Change in count between last two updates
        self.prev_t = ticks_us()
        self.dt = 0 # Amount of time between last two updates

    ## Update the encoder's internal state.
    # This function updates the encoders internal state which calculates current position and velocity.
    # This function must be ran regularly for proper functionality.
    def update(self):
        curr_count = self.tim.counter()
        curr_t = ticks_us()
        dcount = curr_count - self.prev_count
        AR = self.tim.period()
        if dcount > (AR + 1)/2:
            dcount -= (AR + 1)
        elif dcount < -(AR + 1)/2:
            dcount += (AR + 1)
        self.position += dcount #[Ticks]
        self.prev_count = curr_count
        self.delta = dcount # [Ticks]
        self.dt = ticks_diff(curr_t, self.prev_t) # [us]
        self.prev_t = curr_t

    ## Get the relative position of the encoder.
    # This function gets the angular position (in ticks) of the motor relative to the angle at which the encoder
    # was when the encoder object was created.
    def get_position(self):
        return self.position # [Ticks]

    ## Get the velocity of the encoder.
    # This function gets the angular velocity (in ticks / second) of the motor.
    # This function will be less accurate if the encoder is updated too quickly.
    def get_velocity(self):
        return int(self.delta/self.dt * 1000000) # [Ticks/s]

    ## Zeros the encoder.
    # This function zeros the encoder. (Setting position to zero). It first updates the encoder to ensure
    # that the position is set to zero at the moment it is called.
    def zero(self):
        self.update()
        self.position = 0