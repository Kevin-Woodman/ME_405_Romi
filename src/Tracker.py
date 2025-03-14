from pyb import Pin, Timer
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
        self.sectionShare.put(1)

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
