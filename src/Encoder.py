## @file Encoder.py

from time import ticks_us, ticks_diff # Use to get dt value in update()
from pyb import Timer
import math

class Encoder:
    'A quadrature encoder decoding interface encapsulated in a Python class'

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
        self.tim = Timer(tim, period = 0xFFFF, prescaler = 0)
        self.tim.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=Timer.ENC_AB)
        self.position = 0 # Total accumulated position of the encoder
        self.prev_count = 0 # Counter value from the most recent update
        self.delta = 0 # Change in count between last two updates
        self.prev_t = ticks_us()
        self.dt = 0 # Amount of time between last two updates
        
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
        track of the change in count and check for counter reload'''
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

    def get_dt(self):
        '''Returns a measure of velocity using the the most recently updated
        value of delta as determined within the update() method'''
        return self.dt # [us]

    def get_position(self):
        '''Returns the most recently updated value of position as determined
        within the update() method'''
        return self.position # [Ticks]

    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
        value of delta as determined within the update() method'''
        return int(self.delta/self.dt * 1000000) # [Ticks/s]

    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
        to measure with respect to the new zero position'''
        self.update()
        self.position = 0
        
        
        
