""" @file lightSensor.py
 This file contains the driver for the indidual light sensors on the  
 IR Reflectance Sensors
 """

from pyb import ADC

class lightSensor:
    """ Implements multitasking with scheduling and some performance logging.

    This class implements behavior for each individual IR reflectance sesnor. Reading the sensor value and reconfigureing it
    to allow for normalization of the output.
    @b Example:
    @code
        sensor = lightSensor(pyb.Pin.Board.A5)
        
        # Set the white level datum to be 500 
        sensor.setWhiteLevel(500)
        
        # Auto set the black level datum based on what the sensor currently sees
        sensor.setBlackLevel()

        # Read a normalized value from the sensor
        sensor.read()
    @endcode
    """

    
    def __init__(self, pin):
        """ Initialize a lightSensor object
        This function initializes a lightSensor, configuring the provided pin to ADC
        @param pin The pin that the light sensor is connected to.
        """
        self.pin = ADC(pin)
        self.whiteLevel = -1
        self.blackLevel = -1

    
    def _readRaw(self):
        """Read the sensor's ADC pin
        This function reads the raw value off of the ADC pin
        @return raw ADC sensor reading
        """
        return self.pin.read()

    def read(self):
        """ Read the sensor's value
        This function reads the raw value off of the ADC pin and normalizes it based on the set white and black levels.
        If the white or black levels are set they default to 0 and 4,096 respectivly
        @return raw ADC sensor reading
        """
        # Use 0v - 5v as defaults if levels haven't been configured
        whiteLevel = 0 if self.whiteLevel == -1 else self.whiteLevel
        blackLevel = 5 * 4_096 if self.blackLevel == -1 else self.blackLevel

        # Normalize the value we read to have the configured white as 0 and the configured black as 1
        normalizedValue = (self._readRaw() - whiteLevel) / (blackLevel - whiteLevel)
        return normalizedValue

    
    def setWhiteLevel(self, whiteLevel = None):
        """ Set the sensor's white level
        This function sets the white level. If a white level is not provide the sensor reads it's current value to serve
        as the white levle.
        @param whiteLevel desired whiteLevel or None for automatic 
        """
        self.whiteLevel = whiteLevel if whiteLevel else self._readRaw()
        return self.whiteLevel

    
    def setBlackLevel(self, blackLevel = None):
        """ Set the sensor's black level
        This function sets the black level. If a black level is not provide the sensor reads it's current value to serve
        as the black levle.
        @param blacklevle desired blackLevel or None for automatic 
        """
        self.blackLevel = blackLevel if blackLevel else self._readRaw()
        return self.blackLevel
