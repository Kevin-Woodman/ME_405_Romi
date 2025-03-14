from time import ticks_us, ticks_diff

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.lastTime = None
        self.lastError = None
        self.sum = 0


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

    def reset(self):
        self.lastError = None
        self.lastTime = None
        self.sum = 0


