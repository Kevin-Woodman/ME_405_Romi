## @file sensroArray.py
# This file contains the driver for an entire IR reflectance sensor array

import lightSensor
import pyb

class sensorArray:
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

    def getCentroid(self):
        # Centroid = sum of (sensor pos)*(sensor reading) / sum of (sensor readings
        
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