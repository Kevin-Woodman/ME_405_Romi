## @file Controller.py

from imu import IMU
import sensorArray
import PID
from pyb import Pin, Timer, USB_VCP, ADC
import task_share
import cotask

class controller:
    '''
    Controller is the top-level control loop.
    It uses data from whichever sensor is active in the current state to dictate a velocity setpoint to send each motor/encoder combination
    Controller shares line thickness information with Tracker to indicate specific waypoints on the track and supliment Tracker's encoder position tracking to maintain more consistent action.
    '''

    def __init__(self):
        # IMU Configuration
        i2c = pyb.I2C(1, mode=pyb.I2C.CONTROLLER)
        self.imu = IMU(i2c)
        self.imu.changeMode("CONFIGMODE") # Give IMU time to change mode

        # Line Sensor Configuration
        sensorPins = [Pin.board.PB1, Pin.board.PC5, Pin.board.PC0, Pin.board.PA0, Pin.board.PC1, Pin.board.PC4, None, Pin.board.PA7, Pin.board.PA4, Pin.board.PA6, Pin.board.PA1,Pin.board.PC2,Pin.board.PC3]
        lightPins = [Pin.board.PC10, Pin.board.PC11]

        self.sensor = sensorArray.sensorArray(sensorPins, lightPins)
        self.sensor.enable()

        # Line Sensor Calibration

        whiteList =  [2548, 821, 604, 520, 288, 303, 0, 328, 386, 310, 1027, 494, 1934]
        blackList =  [3479, 2297, 2470, 2542, 1974, 1871,0,  1933, 2607, 1908, 3040, 2648, 3454]

        self.sensor.configAll(whiteList,blackList)

        # LINE SENSOR PID CONTROLLER
        # Setpoint
        self.centerPos = 6
        # Feed Forward
        self.defaultVel = 7 # [rad/s]
        # Saturation
        self.maxVel = 19 # [rad/s]
        # Units [centerPos [mm*4] ---> velocity [rad/s]]
        Kp_line = 1.25
        Ki_line = 0.1
        Kd_line = 0

        self.pid_line = PID.PID(Kp_line, Ki_line, Kd_line)

        # IMU Calibration
        with open("calibrationCoefficients.txt", "rb") as file:
            self.imu.writeCoeffs(file.read())
        self.imu.changeMode("IMU")

        self.targetHeading = 0

        # IMU PID CONTROLLER
        # Units [2pi/5760 rads ---> velocity [rad/s]]
        Kp_imu = 0.03 # 25
        Ki_imu = 0.0075 # 5
        Kd_imu = 0

        self.pid_imu = PID.PID(Kp_imu, Ki_imu, Kd_imu)


    def _S0(self):
        '''
        State/Section 0

        Initializes and resets.
        '''
        if (self.state == self.S0_INIT):
            # Wait until enabled
            if(self.enable.get() == 1):
                self.pid_line.reset()
                self.pid_imu.reset()
                self.sensor.enable()
                self.searchThickness = 2

                # Setup target heading to be 180 deg away from inital heading
                self.targetHeading = (self.imu.readEuler()[0]) + 2880
                self.targetHeading -= 5760 if self.targetHeading > 5760 else 0

                self.heading = self.targetHeading

                # -- Reset all flags --
                self.ignoreLine = False
                self.turn1Flag = False
                self.turn2Flag = False
                self.wallTurn1Flag = False
                self.wallTurn2Flag = False
                self.imuResetFlag = False
                self.Dir = 1

                # Set inital states
                self.state = self.S1_CONTROL
                self.section = self.SC1

    def _SC1(self):
        '''
        Section 1

        Control/Sense loop from start to post-diamond.

        - Controls based on line sensor
        - Looks for thick line
            - Sends first thick line instance to Tracker
            - Switches to driving forward
        - State gets switched to S2_Diamond after indicated by Tracker share
        '''

        if (self.state == self.S1_CONTROL):
            # If disabled
            if (self.enable.get() == 0):
                self.rVelShare.put(0)
                self.lVelShare.put(0)
                self.sensor.disable()
                self.state = self.S0_INIT
                return
            elif(self.sectionShare.get() == 2):
                self.ignoreLine = False
                self.searchThickness = -1 # No search thickness
            elif(self.sectionShare.get() == 3):
                self.searchThickness = 2 # Search for no line

            if (not self.ignoreLine):
                error = self.centerPos - self.centroidPos
                control = self.pid_line.update(error)

                self.velR =  (self.defaultVel + control)
                if (self.velR < -self.maxVel): self.velR = -self.maxVel
                if (self.velR > self.maxVel): self.velR = self.maxVel

                self.velL = (self.defaultVel - control)
                if (self.velL < -self.maxVel): self.velL = -self.maxVel
                if (self.velL > self.maxVel): self.velL = self.maxVel

                self.rVelShare.put(self.velR)
                self.lVelShare.put(self.velL)
            else:
                self.rVelShare.put(self.defaultVel - 1)
                self.lVelShare.put(self.defaultVel + 1)

            self.state = self.S2_SENSE


        elif (self.state == self.S2_SENSE):
            self.centroidPos, thickness = self.sensor.getCentroid()
            if(thickness == self.searchThickness):
                if self.sectionShare.get() == 1:
                    self.ignoreLine = True
                    self.pid_line.reset()
                    self.sectionShare.put(-1)
                if self.sectionShare.get() == 3:
                    self.section = self.SC2
                    self.sectionShare.put(-2)
                if self.sectionShare.get() == 8:
                    self.enable.put(0)

            if(thickness == 0):
                # We don't have enough readings, this will set error to 0 so just Kp will not turn the bot.
                self.centroidPos = self.centerPos
            self.state = self.S1_CONTROL

    def _SC2(self):
        '''
        Section 2

        Control/Sense Loop for IMU section

        -
        '''
        if (self.state == self.S1_CONTROL):
            if (self.enable.get() == 0 and not self.turn2Flag):
                self.sectionShare.put(-3)
                self.targetHeading -= 1440
                self.targetHeading += 5760 if self.targetHeading < 0 else 0
                self.turn2Flag = True
                self.imuResetFlag = False
                self.Dir = -1 # Reverse Romi
                self.enable.put(1)

            elif(self.sectionShare.get() == 4 and not self.turn1Flag):
                self.targetHeading += 1440
                self.targetHeading -= 5760 if self.targetHeading > 5760 else 0
                self.turn1Flag = True
                self.imuResetFlag = False

            elif(self.sectionShare.get() == 6 and not self.wallTurn1Flag):
                self.targetHeading -= 1440
                self.targetHeading += 5760 if self.targetHeading < 0 else 0
                self.wallTurn1Flag = True
                self.imuResetFlag = False

            elif(self.sectionShare.get() == 7 and not self.wallTurn2Flag):
                self.targetHeading -= 1440
                self.targetHeading += 5760 if self.targetHeading < 0 else 0
                self.wallTurn2Flag = True
                self.imuResetFlag = False

            elif(self.sectionShare.get() == 8 ):
                self.rVelShare.put(0)
                self.lVelShare.put(0)
                self.state = self.S0_INIT
                return

            # This weird calculation compensates for circular angles to solve for the shortest error to North
            # i.e. When Romi is at 0 heading but needs to go to 5319, it will see the error as 441 not 5319.
            error1 = self.targetHeading - self.heading
            error2 = error1 + 5760
            error3 = error1 - 5760

            error = min(abs(error1),abs(error2),abs(error3))
            for e in [error1,error2,error3]:
                if(error == abs(e)):
                    error = e
                    break

            if abs(error) <= 75 and not self.imuResetFlag:
                self.imuResetFlag = True
                self.pid_imu.reset()

            control = self.pid_imu.update(error)

            velR = (6 * self.Dir) - control
            self.maxVel = 7.5
            if (velR < -self.maxVel): velR = -self.maxVel
            if (velR > self.maxVel): velR = self.maxVel

            velL = (6 * self.Dir)  + control
            if (velL < -self.maxVel): velL = -self.maxVel
            if (velL > self.maxVel): velL = self.maxVel

            self.rVelShare.put(velR)
            self.lVelShare.put(velL)
            self.state = self.S2_SENSE

        elif (self.state == self.S2_SENSE):
            self.heading = self.imu.readEuler()[0]
            self.state = self.S1_CONTROL

    def task(self, shares):

        self.enable, self.lVelShare, self.rVelShare, self.sectionShare = shares

        #Declaring States, categorized by section functions
        self.S0_INIT = 0

        self.SC1 = 1
        self.S1_CONTROL = 1
        self.S2_SENSE = 2

        self.SC2 = 2

        self.state = self.S0_INIT
        self.section = self.SC1


        self.centroidPos = self.centerPos

        self.ignoreLine = False

        self.velR = self.defaultVel
        self.velL = self.defaultVel

        while True:

            if (self.state == self.S0_INIT):
                self._S0()

            elif (self.section == self.SC1):
                self._SC1()

            elif (self.section == self.SC2):
                self._SC2()

            yield self.section

