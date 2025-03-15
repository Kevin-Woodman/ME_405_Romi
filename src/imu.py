## @file imu.py
# This file contains a driver for the LSM6DS33 inertial measurement unit.

from pyb import I2C
from struct import unpack

## Implements a driver for the LSM6DS33 inertial measurement unit.
# This class also for control of certain aspects of the LSM6DS33 IMU chip over a I2C bus.
# This class requires a configured I2C controller object.
# @b Example:
# @code
# # Create an I2C controller object
# i2c = pyb.I2C(1, mode=pyb.I2C.CONTROLLER)
# # Create an IMU object
# imu = IMU(i2c)
# # Change the IMU mode to configure
# imu.changeMode("CONFIGMODE")
#
# # <Run other code while imu changes mode>
#
# # Write stored config values into the IMU
# with open("calibrationCoefficients.txt", "rb") as file:
#    imu.writeCoeffs(file.read())
# # Change the IMU into an operating mode
# imu.changeMode("IMU")
# @endcode
class IMU():

    deviceID = 0x28

    ## This dictionary contains the IMUs operational modes.
    # This dictionary can be used to convert a text mode into the proper bit string needed to change the mode of the IMU.
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

    ## Initializes an IMU objet.
    # This function creates an IMU object using a preconfigured I2C controller.
    # @code
    # # Create an I2C controller object
    # i2c = pyb.I2C(1, mode=pyb.I2C.CONTROLLER)
    # # Create an IMU object
    # imu = IMU(i2c)
    # @endcode
    # @param I2C preconfigured I2C controller object
    def __init__(self, I2C):
        self.I2C = I2C

    ## Change the mode of the IMU.
    # This function changes the mode of the IMU based on a provided string.
    # If the string is not a valid mode no change is made. Note that te IMU takes time to switch between modes.
    # @param mode A mode string contained in opModes
    def changeMode(self, mode):
        # OPR_MODE ID = 0x3D
        # Defaults to Config Mode

        if mode in self.opModes:
            self.I2C.mem_write(self.opModes[mode], self.deviceID, 0x3D)
        else:
            raise ValueError("Invalid Mode Name")

    ## Check the calibration status of the IMU.
    # This function checks if the 'system calibrated' bits of the CALIB_STAT register are set.
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

    ## Reads the configuration coefficients.
    # This function reads the calibration coefficients from the IMU. The IMU @b must
    # be in "CONFIGMODE"
    def readCoeffs(self):
        coeffs = bytearray((0 for n in range(22)))
        self.I2C.mem_read(coeffs, self.deviceID, 0x55)
        return coeffs

    ## Writes the configuration coefficients.
    # This function writes the calibration coefficients to the IMU. The IMU @b must
    # be in "CONFIGMODE"
    # @param coeffs A bytes or byteArray object holding the calibration coefficients.
    def writeCoeffs(self, coeffs):
        self.I2C.mem_write(coeffs, self.deviceID, 0x55)

    ## Reads the Euler angles.
    # This function reads the euler angles out of the IMU. Must not be in "CONFIGMODE"
    # The angles are returned as a tuple of (yaw, roll, pitch)
    def readEuler(self):
        eulerID = 0x1A
        euler = bytearray((0 for n in range(6)))
        self.I2C.mem_read(euler, self.deviceID, eulerID)
        # LSB first, little-endian = <
        yaw, roll, pitch = unpack("<hhh", euler)
        return yaw, roll, pitch

    ## Reads the Angular velocity.
    # This function reads the angular velocity out of the IMU. Must not be in "CONFIGMODE"
    # The angular velocities are returned as a tuple of (yawRate, rollRate, pitchRate)
    def readAngVel(self):
        gyrID = 0x14
        gyr = bytearray((0 in range(6)))
        self.I2C.mem_write(gyr, self.deviceID, gyrID)
        # LSB first, little-endian = <
        yawRate, rollRate, pitchRate = unpack("<hhh", gyr)
        return yawRate, rollRate, pitchRate
