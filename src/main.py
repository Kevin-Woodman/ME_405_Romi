import sensorArray
import PID
import Motor
import Encoder
from pyb import Pin, Timer, USB_VCP, ADC
import task_share
import cotask
import gc
from imu import IMU

class Tracker:
    '''
    Tracker takes wheel angular position shares from the R/L motor tasks and calculates the relative position of Romi's center.
    
    This information is used to communicate to the controller task what "section" of the track Romi is in so that specific controlling can be performed.

    Tracker also recieves information from the controller task that helps indicate specific landmarks on the track and reset the encoders so that more accurate tracking is conducted.

    Additionally, Tracker controls the servo motion directly in the task.
    '''

    def __init__(self):
        # Servo Configuration
        '''
        Pulse width 10000 = 90deg = up
        Pulse width 14500 = 180deg = down
        '''
        tim4 = Timer(4, freq=300) # Tim freq of 300Hz corresponds to 3333us period.
        self.servo = tim4.channel(2, pin = Pin.board.PB7, mode = Timer.PWM, pulse_width = 10000)


    def _S0(self):
        '''
        Initialization state.

        Sets the servo to upright position and resets the encoders
        '''

        self.servo.pulse_width(10000)
        self.resetR.put(1)
        self.resetL.put(1)
        self.state = self.S1_START
        #self.state = self.S2_POST_DIAMOND
        self.sectionShare.put(1)
        #self.sectionShare.put(3)



    def _S1(self):
        '''
        First section of the track up until the diamond.

        Uses Encoder reading to swing servo and get the first cup.
        '''
        if(self.sectionShare.get() == -1):
            self.resetR.put(1)
            self.resetL.put(1)
            self.state = self.S2_POST_DIAMOND
            return

        # Calculating Position
        rPos = self.posR.get()
        lPos = self.posL.get()
        avg = (rPos + lPos) / 2
        #print("L:", lPos, "R:", rPos, "Avg:", avg)

        # Controlling Servo
        if avg >= 100_000: pass
        elif avg >= 4_100:
            self.servo.pulse_width(10000)
        elif avg >= 500:
            self.servo.pulse_width(14500)


    def _S2(self):
        '''
        Diamond-onward state.

        - Resets encoder ticks.
        - Sends section change to controller after Diamond Ticks
        - Deploys arm after certain ticks
            - Undeploy
        '''

        if(self.sectionShare.get() == -2):
            self.resetR.put(1)
            self.resetL.put(1)
            self.state = self.S3_IMU
            return

        rPos = self.posR.get()
        lPos = self.posL.get()
        avg = (rPos + lPos) / 2
        #print("L:", lPos, "R:", rPos, "Avg:", avg)
        if avg >= 100_000: pass
        elif avg >= 17_000:
            self.sectionShare.put(3)
        elif avg >= 5_250:
            self.servo.pulse_width(10000)
        elif avg >= 4_000:
            self.servo.pulse_width(14500)
            pass
        elif avg >= 850:
            self.sectionShare.put(2)
            pass


    def _S3(self):
        '''
        IMU state.

        - Reset encoder ticks
        - Count until turn
            - Send flag to control after turn
        '''

        if(self.sectionShare.get() == -3):
            self.resetR.put(1)
            self.resetL.put(1)
            self.state = self.S4_WALL
            return

        rPos = self.posR.get()
        lPos = self.posL.get()
        avg = ((rPos + lPos) / 2) 
        #print("L:", lPos, "R:", rPos, "Avg:", avg)
        if avg >= 8_500: pass #Seems to be falling through for some reason
        elif avg >= 4_475:
            self.sectionShare.put(4)


    def _S4(self):
        '''
        Finish State

        - sequence of finishing IMU moves to get to Line
        '''
        rPos =  4294967295 - self.posR.get()
        lPos =  4294967295 - self.posL.get()

        avg = (rPos + lPos) / 2
        print(avg)

        if avg >= 8_000: #Seems to be falling through for some reason
            pass
        elif avg >= 4_850:
            self.enable.put(0)
            self.sectionShare.put(8)
            self.state = self.S0_INIT
            return
        elif avg >= 2_950:
            self.sectionShare.put(7)
        elif avg >= 1_500:
            self.sectionShare.put(6)
        
    def _S5(self):
        pass
      

    def task(self, shares):
        '''
        Task is the main function that is called by the scehduler.

        Task unpacks shares and declares them as class parameters, declares the states and then simply checks the state and calls the corresponds state function.
        '''

        # Unpacking Shares
        self.enable, self.sectionShare, self.posL, self.posR, self.resetL, self.resetR = shares

        # Declaring States
        self.S0_INIT = 0
        self.S1_START = 1 
        self.S2_POST_DIAMOND = 2 
        self.S3_IMU = 3
        self.S4_WALL = 4
        self.S5_FINISH = 5

        self.state = self.S0_INIT

        # Task FSM Loop
        while True:
            if(self.enable.get() == 0 and not (self.state == self.S4_WALL or self.state == self.S3_IMU or self.state == self.S5_FINISH)):
                self._S0()
                
            if(self.state == self.S1_START):
                self._S1()

            elif (self.state == self.S2_POST_DIAMOND):
                self._S2()

            elif (self.state == self.S3_IMU):
                self._S3()

            elif (self.state == self.S4_WALL):
                self._S4()

            elif (self.state == self.S5_FINISH):
                self._S5()

            yield self.state

    


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
        self.imu.changeMode("CONFIGMODE") #Give IMU time to change mode

        # Line Sensor Configuration
        sensorPins = [Pin.board.PB1, Pin.board.PC5, Pin.board.PC0, Pin.board.PA0, Pin.board.PC1, Pin.board.PC4, None, Pin.board.PA7, Pin.board.PA4, Pin.board.PA6, Pin.board.PA1,Pin.board.PC2,Pin.board.PC3]
        lightPins = [Pin.board.PC10, Pin.board.PC11]

        self.sensor = sensorArray.sensorArray(sensorPins, lightPins)
        self.sensor.enable()

        # Line Sensor Calibration	

        whiteList =  [2548, 821, 604, 520, 288, 303, 0, 328, 386, 310, 1027, 494, 1934] 
        blackList =  [3479, 2297, 2470, 2542, 1974, 1871,0,  1933, 2607, 1908, 3040, 2648, 3454]

        self.sensor.configAll(whiteList,blackList)
        #self.sensor.configAll()

        # LINE SENSOR PID CONTROLLER
            # Setpoint
        self.centerPos = 6
            # Feed Forward
        self.defaultEff = 7 # [rad/s]
            # Saturation
        self.maxEff = 19 # [rad/s]
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
                self.ignoreLine = False
                self.searchThickness = 2
                self.sensor.enable()
                self.centerPos = 6
                self.defaultEff = 7 # [rad/s]
                 # Saturation
                self.maxEff = 19 # [rad/s]
                self.turn1Flag = False
                self.turn2Flag = False
                self.wallTurn1Flag = False
                self.wallTurn2Flag = False
                self.imuResetFlag = False
                self.Dir = 1

                # Setup target heading to be 180 deg away from inital heading
                self.targetHeading = (self.imu.readEuler()[0]) + 2880
                self.targetHeading -= 5760 if self.targetHeading > 5760 else 0 

                self.heading = self.targetHeading

                #self.targetHeading = (self.imu.readEuler()[0])
                
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
                self.rEffShare.put(0)
                self.lEffShare.put(0)
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

                self.effR =  (self.defaultEff + control)
                if (self.effR < -self.maxEff): self.effR = -self.maxEff
                if (self.effR > self.maxEff): self.effR = self.maxEff

                self.effL = (self.defaultEff - control)
                if (self.effL < -self.maxEff): self.effL = -self.maxEff
                if (self.effL > self.maxEff): self.effL = self.maxEff

                self.rEffShare.put(self.effR)
                self.lEffShare.put(self.effL)
            else:
                self.rEffShare.put(self.defaultEff - 1)
                self.lEffShare.put(self.defaultEff + 1)

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
                # We can do what ever we want, not enough readings
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
                self.Dir = -1
                self.enable.put(1)

                # self.rEffShare.put(0)
                # self.lEffShare.put(0)
                # self.state = self.S0_INIT
                # return
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
                self.rEffShare.put(0)
                self.lEffShare.put(0)
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
    
            effR = (6 * self.Dir) - control
            self.maxEff = 7.5
            if (effR < -self.maxEff): effR = -self.maxEff
            if (effR > self.maxEff): effR = self.maxEff

            effL = (6 * self.Dir)  + control
            if (effL < -self.maxEff): effL = -self.maxEff
            if (effL > self.maxEff): effL = self.maxEff

            self.rEffShare.put(effR)
            self.lEffShare.put(effL)
            self.state = self.S2_SENSE

        elif (self.state == self.S2_SENSE):
            self.heading = self.imu.readEuler()[0]
            self.state = self.S1_CONTROL


    def _SC4(self):
        '''
        Section 4

        Control/Sense Loop for...
        '''
        pass


    def task(self, shares):

        self.enable, self.lEffShare, self.rEffShare, self.sectionShare = shares

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

        self.effR = self.defaultEff
        self.effL = self.defaultEff

        while True:

            if (self.state == self.S0_INIT):
                self._S0()

            elif (self.section == self.SC1):
                self._SC1()

            elif (self.section == self.SC2):
                self._SC2()
        
            yield self.section




class MotorEncoder():
    def __init__(self, side, vbat):

        self.vbat = vbat
        tm2 = Timer(2, freq=50*100000)

        if side == 'R':
            self.motor = Motor.Motor((tm2, 1, Pin.board.PA15), Pin.board.PH0, Pin.board.PH1)
            
            self.encoder = Encoder.Encoder(1, Pin.board.PA8, Pin.board.PA9)
            self.name = "Right"

        elif side == 'L':
            self.motor = Motor.Motor((tm2, 3, Pin.board.PB10), Pin.board.PB3, Pin.board.PA10)
            self.encoder = Encoder.Encoder(3, Pin.board.PB4, Pin.board.PB5)
            self.name = "Left"
        else:
            raise ValueError("Class takes 'R' or 'L' as parameters")

        self.vel2effort = lambda vel: ((vel / 5.57) + 2.1 * (-1 if vel < 0 else 1)) 

        # (DeltaVelocity [rad/s] ----> DeltaEffort [%])
        Kp_m = 0.5  # 1.5# 2.5 # Initial guess based on pseudo-linear relationship of +10% effort = +3.5 rad/s
        Ki_m = 0.25 # 0.25 #0.5 #0.5
        Kd_m = 0

        self.pid = PID.PID(Kp_m, Ki_m, Kd_m)
        self.error = 0

    def task(self, shares):

        velocityShare, pos, reset = shares

        S0_INIT = 0
        S1_ACTUATE = 1
        S2_SENSE = 2

        state = S0_INIT

        while True:
            
            if (state == S0_INIT):
                self.motor.enable()
                self.encoder.update()

                state = S1_ACTUATE

            elif (state == S1_ACTUATE):
                if (velocityShare.get() == 0):
                    self.motor.set_effort(0)
                else:
                    voltageDelta = self.pid.update(self.error)
                    voltage = voltageDelta + self.vel2effort(velocityShare.get())
                    self.motor.set_effort(voltage/Vbat * 100)
                    #print(self.name,voltage/Vbat * 100)

                state = S2_SENSE


            elif (state == S2_SENSE):
                if (reset.get()):
                    self.encoder.zero()
                    reset.put(0)
                else:  # Else so we don't double update
                    self.encoder.update()

                self.error = velocityShare.get() - (self.encoder.get_velocity() * 2 * 3.1415 / 1440)

                pos.put(self.encoder.get_position())
                state = S1_ACTUATE

                #print(self.name, "Goal:", velocityShare.get(), "Actual:",self.encoder.get_velocity()  * 2 * 3.1415 / 1440)

            else:
                raise ValueError('Invalid state')

            yield state




if __name__ == '__main__':
    # Bluetooth Configuration
    uart = pyb.UART(1,115200)
    uart.init(115200, bits=8, parity=None, stop=1)
    pyb.repl_uart(uart)

    batPin = ADC(Pin.board.PB0)
    
    print("Bat Pin", batPin.read())

    Vbat = batPin.read()*(3.3/4095)*(58/20)*1.125

    if(Vbat <= 6.65):
        # If we are low on charge
        print("CHARGE BATTERIES")
        exit(0)
   
    motorR = MotorEncoder("R",batPin)
    motorL = MotorEncoder("L",batPin)

    controller = controller()
    tracker = Tracker()

    enabled = task_share.Share('B', thread_protect=False, name="enabled")
    effortL = task_share.Share('f', thread_protect=False, name="effortL")
    effortR = task_share.Share('f', thread_protect=False, name="effortR")

    posL = task_share.Share('L', thread_protect=False, name="posL")
    posR = task_share.Share('L', thread_protect=False, name="posR")

    sectionShare = task_share.Share('b', thread_protect=False, name="section")

    encoderResetL = task_share.Share('B', thread_protect=False, name="resetL")
    encoderResetR =  task_share.Share('B', thread_protect=False, name="resetR")

    # User_task = cotask.Task(User, name="User", priority=1, period=100, profile=True, trace=False, shares=(enabled))
    Control_task = cotask.Task(controller.task, name="User", priority=2, period=10, profile=True, trace=False,
                            shares=(enabled, effortL, effortR, sectionShare))

    MotorR_task = cotask.Task(motorR.task, name="Drive", priority=3, period=5, profile=True, trace=False,
                             shares=(effortR, posR, encoderResetR))

    MotorL_task = cotask.Task(motorL.task, name="Drive", priority=3, period=5, profile=True, trace=False,
                              shares=(effortL, posL, encoderResetL))

    Tracker_task = cotask.Task(tracker.task, name="Tracker", priority=1, period = 20, shares= (enabled, sectionShare, posL, posR, encoderResetL, encoderResetR))

    # cotask.task_list.append(User_task)
    cotask.task_list.append(Control_task)
    cotask.task_list.append(MotorR_task)
    cotask.task_list.append(MotorL_task)
    cotask.task_list.append(Tracker_task)

    gc.collect()

    # START/STOP Button Config
    pyb.ExtInt(Pin.cpu.C13, pyb.ExtInt.IRQ_FALLING, Pin.PULL_NONE, lambda b: enabled.put(0 if enabled.get() else 1))

    bumpSensors = [Pin.board.PB11,Pin.board.PB14,Pin.board.PB15]

    for bump in bumpSensors:
        pyb.ExtInt(bump, pyb.ExtInt.IRQ_FALLING, Pin.PULL_UP, lambda b: enabled.put(0))

    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
