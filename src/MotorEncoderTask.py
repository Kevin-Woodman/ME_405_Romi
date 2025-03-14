from pyb import Pin, Timer, USB_VCP, ADC
import task_share
import cotask

import PID
import Motor
import Encoder
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

        self.vel2volt = lambda vel: ((vel / 5.57) + 2.1 * (-1 if vel < 0 else 1))

        # (DeltaVelocity [rad/s] ----> DeltaVoltage [V])
        Kp_m = 0.5
        Ki_m = 0.25
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
                    voltage = voltageDelta + self.vel2volt(velocityShare.get())
                    self.motor.set_effort(voltage/Vbat * 100)

                state = S2_SENSE


            elif (state == S2_SENSE):
                if (reset.get()):
                    self.encoder.zero()
                    reset.put(0)
                else:  # else so we don't double update
                    self.encoder.update()

                self.error = velocityShare.get() - (self.encoder.get_velocity() * 2 * 3.1415 / 1440)

                pos.put(self.encoder.get_position())
                state = S1_ACTUATE

            else:
                raise ValueError('Invalid state')

            yield state