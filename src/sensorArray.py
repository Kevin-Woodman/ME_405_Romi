## @file sensroArray.py
# This file contains the driver for an entire IR reflectance sensor array

import lightSensor
import pyb

## Implements Light sensor array behavior.
# This class implements behavior for the whole IR reflectance sensor. Calculating line thickness and centroid of the line.
# It also allows reconfiguring of every individual light sensor and control of the IR emitters
# @b Example:
# @code
# # Line Sensor Configuration
# sensorPins = [Pin.board.PB1, Pin.board.PC5, Pin.board.PC0, Pin.board.PA0, Pin.board.PC1, Pin.board.PC4, None, Pin.board.PA7, Pin.board.PA4, Pin.board.PA6, Pin.board.PA1, Pin.board.PC2, Pin.board.PC3]
# lightPins = [Pin.board.PC10, Pin.board.PC11]
#
# # Create the sensor object
# sensor = sensorArray(sensorPins, lightPins)
#
# # Enable the IR emitters
# sensor.enable()
#
# # Auto configuration process
# sensor.configAll()
#
# # Manual configuration process
# whiteList = [2548, 821, 604, 520, 288, 303, 0, 328, 386, 310, 1027, 494, 1934]
# blackList = [3479, 2297, 2470, 2542, 1974, 1871, 0, 1933, 2607, 1908, 3040, 2648, 3454]
#
# sensor.configAll(whiteList, blackList)
# @endcode
class sensorArray:

    ## Initialize a sensorArray object.
    # This function initializes a sensorArray with the given sensor pins and emitter pins. Configures emitter pins to push-pull output
    # @param sensorPins list of pins to initialize sensors to. Can contain None if you are missing sensor index. Must be ADC compatible pins
    # @param lightPins list of pins for the IR emitters
    def __init__(self, sensorPins, lightPins):
        # Initalize array of lightSensor objects based on pins
        self.sensors = []
        for pin in sensorPins:
            if not pin: 
                self.sensors.append(None)
            else:
                sensor = lightSensor.lightSensor(pin)
                self.sensors.append(sensor)

        self.lights = [pyb.Pin(pin, pyb.Pin.OUT_PP) for pin in lightPins]

    ## Configures all of the sensors
    # This function configures all of the sensor's white and black levels either automatically or manually
    # @param whiteList if a whiteList is not provided the sensor will automatically calibrate when 'enter' is hit
    # @param blackList if a blackList is not provided the sensor will automatically calibrate when 'enter' is hit
    def configAll(self, whiteList = None, blackList = None):
        whiteLevels = []
        blackLevels = []

        if not whiteList:
            input("Place sensor over white surface and hit enter")

        for i, sensor in enumerate(self.sensors):
            if not sensor: continue
            whiteLevels.append(sensor.setWhiteLevel(whiteList[i] if whiteList else None))

        if not blackList:
            input("Place sensor over black surface and hit enter")
        for i,sensor in enumerate(self.sensors):
            if not sensor: continue
            blackLevels.append(sensor.setBlackLevel(blackList[i] if blackList else None))
   
        print("White:", whiteLevels,"Black:",blackLevels)

    ## Calculates the centroid of the line sensor's reading
    # The centroid of the line is calculated by taking a weighted average of each sensor's weighted by its index.
    # \image html Sumval.png width=30%
    # \image html centroid.png width=40%
    # This function also returns a thickness value: 0 if sumVal <= 1.0, 2 if sumVal >= 6.5, or 1 otherwise.
    # This indicates the thickness of the line.
    def getCentroid(self):
        # Centroid = sum of (sensor pos)*(sensor reading) / sum of (sensor readings)
        
        centroid = 0
        sumVal = 0

        for i, sensor in enumerate(self.sensors):
            if not sensor: continue
            val = sensor.read()
            sumVal += val
            centroid += (i + 1)*val


        thickness = 1
        if(sumVal <= 1.0): # Too faint reading to reliably get centroid
            return -1, 0
        elif(sumVal >= 6.5): # Thick line
            thickness = 2

        centroid /= sumVal
        return centroid, thickness

    ## Enable IR emmiters
    # Enables all the IR emmiters to full power
    def enable(self):
        for pin in self.lights:
            pin.high()

    ## Disable IR emmiters
    # Disables all the IR emmiters
    def disable(self):
        for pin in self.lights:
            pin.low()