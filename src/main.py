## @file main.py
# This file contains the main program which Romi will run on startup and reset. It includes 4 tasks.
# Task Name  | Task Function | Task Priority | Task Period [us]
# ------------- | ------------- | ------------- | -------------
# Control  | Controller.Controller.task | 2 | 10
# Tracker  | Tracker.Tracker.task | 1 | 20
# DriveR  | MotorEncoderTask.MotorEncoder.task | 3 | 5
# DriveL  | MotorEncoderTask.MotorEncoder.task| 3 | 5
# This file also contains interrupt configuration to allow the bump sensors to turn Romi on or off.
# @code
# bumpSensors = [Pin.board.PB11, Pin.board.PB14, Pin.board.PB15]
# for bump in bumpSensors:
#     pyb.ExtInt(bump, pyb.ExtInt.IRQ_FALLING, Pin.PULL_UP, lambda b: enabled.put(0))
# @endcode

from pyb import Pin, Timer, USB_VCP, ADC
import task_share
import cotask
import gc
from Controller import controller
from Tracker import Tracker
from MotorEncoderTask import MotorEncoder

if __name__ == '__main__':
    # Bluetooth Configuration
    uart = pyb.UART(5,115200)
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
    velocityL = task_share.Share('f', thread_protect=False, name="velocityL")
    velocityR = task_share.Share('f', thread_protect=False, name="velocityR")

    posL = task_share.Share('L', thread_protect=False, name="posL")
    posR = task_share.Share('L', thread_protect=False, name="posR")

    sectionShare = task_share.Share('b', thread_protect=False, name="section")

    encoderResetL = task_share.Share('B', thread_protect=False, name="resetL")
    encoderResetR =  task_share.Share('B', thread_protect=False, name="resetR")

    # User_task = cotask.Task(User, name="User", priority=1, period=100, profile=True, trace=False, shares=(enabled))
    Control_task = cotask.Task(controller.task, name="Control", priority=2, period=10, profile=True, trace=False, shares=(enabled, velocityL, velocityR, sectionShare))

    MotorR_task = cotask.Task(motorR.task, name="DriveR", priority=3, period=5, profile=True, trace=False, shares=(velocityR, posR, encoderResetR))

    MotorL_task = cotask.Task(motorL.task, name="DriveL", priority=3, period=5, profile=True, trace=False, shares=(velocityL, posL, encoderResetL))

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
