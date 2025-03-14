from pyb import I2C
from struct import unpack

class IMU():

    deviceID = 0x28

    opModes = {
    "CONFIGMODE": 0b0000,
    "ACCONLY": 0b0001,
    "MAGONLY": 0b0010,
    "GYROONLY": 0b0011,
    "ACCMAG": 0b0100,
    "ACCGYRO": 0b0101,
    "MAGGYRO": 0b0110,
    "AMG": 0b0111,
    "IMU": 0b1000,
    "COMPASS": 0b1001,
    "M4G": 0b1010,
    "NDOF_FMC_OFF": 0b1011,
    "NDOF": 0b1100
    }

    def __init__(self, I2C):
        self.I2C = I2C

    def changeMode(self, mode):
        # OPR_MODE ID = 0x3D
        # Defaults to Config Mode

        if mode in self.opModes:
            self.I2C.mem_write(self.opModes[mode], self.deviceID, 0x3D)
        else:
            raise ValueError("Invalid Mode Name")

    def calibStat(self):
        # CALIB_STAT ID = 0x35
        # Retrieve and parse calibration status from IMU

        buf = bytearray((0, )) # I honestly don't know why we need this "0, "
        self.I2C.mem_read(buf, self.deviceID, 0x35)
        print(bin(buf[0])) # This unpacks the bytearray and prints in bits
        if buf[0] == 0xFF:
            print("IMU Fully Calibrated")
            return True
        else:
            print("IMU Not Fully Calibrated")
            return False

    def readCoeffs(self):
        coeffs = bytearray((0 for n in range(22)))
        self.I2C.mem_read(coeffs, self.deviceID, 0x55)
        return coeffs

    def writeCoeffs(self, coeffs):
        self.I2C.mem_write(coeffs, self.deviceID, 0x55)

    def readEuler(self):
        eulerID = 0x1A
        euler = bytearray((0 for n in range(6)))
        self.I2C.mem_read(euler, self.deviceID, eulerID)
        # LSB first, little-endian = <
        yaw, roll, pitch = unpack("<hhh", euler)
        return yaw, roll, pitch

    def readAngVel(self):
        gyrID = 0x14
        gyr = bytearray((0 in range(6)))
        self.I2C.mem_write(gyr, self.deviceID, gyrID)
        # LSB first, little-endian = <
        yawRate, rollRate, pitchRate = unpack("<hhh", gyr)
        return yawRate, rollRate, pitchRate
