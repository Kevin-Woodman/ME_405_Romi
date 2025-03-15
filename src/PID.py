## @file PID.py
# This file contains the class for a PID controller

from time import ticks_us, ticks_diff

## Implementation of a PID controller.
# PID output is given by:
# \image html PID.png width=50%
# @b Example:
# @code
# # Create a PID object
# pid = PID(1, 0.2, 0)
#
# # Update the PID object
# pid.update(error)
#
# # Reset the PID object
# pid.reset()

# @endcode
class PID:

    ## Initialize a PID object.
    # Initialize the PID object with provided kp, ki, and kd.
    # @param kp proportional control
    # @param ki integral control
    # @param kd derivative control
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.lastTime = None
        self.lastError = None
        self.sum = 0

    ## Update the PID control with a given error
    # Returns an output based on the provided error and the previous errors.
    # This function takes into account the time between when it was last called.
    # The output is given by: \image html PID.png width=50%
    # @param error The current error to feed into the PID.
    def update(self, error):
        if(self.lastTime == None): 
            self.lastError = error
            self.lastTime = ticks_us()
            return 0

        deltaT = ticks_diff(ticks_us(), self.lastTime) / 1_000_000

        derivative = (error - self.lastError) / deltaT if self.lastError else 0

        self.lastError = error
        self.lastTime = ticks_us()

        self.sum += deltaT * error
        output = self.kp * error + self.ki * self.sum + self.kd * derivative

        return output

    ## Resets the pid object
    # Sets the integral and next derivative to 0
    def reset(self):
        self.lastError = None
        self.lastTime = None
        self.sum = 0


