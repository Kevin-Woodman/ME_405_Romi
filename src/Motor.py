from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
    motor drivers using separate PWM and direction inputs such as the DRV8838
    drivers present on the Romi chassis from Pololu.'''

    def __init__(self, PWM, DIR, nSLP):
        '''Initializes a Motor object'''
        self.PWM_pin = PWM[0].channel(PWM[1], pin=PWM[2], mode=Timer.PWM, pulse_width_percent=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        pass

    def set_effort(self, effort):
        '''Sets the present effort requested from the motor based on an input value
        between -100 and 100'''
        if effort < 0:
            self.DIR_pin.high()
            self.PWM_pin.pulse_width_percent(-effort)
        else:
            self.DIR_pin.low()
            self.PWM_pin.pulse_width_percent(effort)
        pass

    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.PWM_pin.pulse_width_percent(0)
        self.nSLP_pin.high()
        pass
   
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()

if __name__ == "__main__":
    pin = Pin.board.PA15
    print(pin.af_list())