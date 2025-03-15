## @file mainpage.py
# @author Kevin Woodman
# @author Nick Larson
# @mainpage
# @section ss_intro Introduction
# The Romi Project tasked teams of two students with developing a Micropython-based, autonomous, two-wheel-drive
# robot which competed in time-trials of a fixed track. The project exercised a holistic view of the mechatronics
# discipline, incorporating elements of mechanical design, electrical engineering, computer science, and control theory.
# Each team was given equal foundation for development: a Romi Chassis Kit equipped with a Motor Driver and Power Distribution
# Board from Pololu Robotics, and a Nucleo-L476RG development board from ST electronics connected to a custom extension board,
# the Shoe of Brian.
#
# [Hardware Render]
#
# The game track, shown in a birds-eye view below, provided a diverse array of obstacles for Romi to navigate.
#
# \image html "BlankGameTrack.jpg" width=75%
# Deliberately, few rules were outlined to allow for creative solutions to the course’s challenges. Romi robots were
# required to begin within the 2D confines of the grey starting square and contact all checkpoints (denoted by labeled black dots)
# in numbered order before returning at least their geometric center to within the starting square to complete a time trial.
# The only additional rule required that the robot detect the wall adjacent to the starting square at some point during the run.
# Finally, two zones housed an object that, if displaced outside of the dotted regions, would yield a five second time deduction
# each for the trial.
#
# @section ss_hardware Hardware
# The team’s Romi affectionately nicknamed “Zippy,” was developed in ten weeks and includes a 9-DOF IMU, 13-unit IR
# reflectance sensor array, tactile switch bump sensor, HC-05 Bluetooth module, and 20kg positional servo, in addition
# to the provided MCU and Motor Driver/PDB development boards.
#
# [Romi Photo]
#
# This section serves to break down each hardware component individually and provides a brief description
# of the purpose and implementation of each.
#
# @subsection ss_wiring_diagram Wiring Diagram
# \image html "Final Wiring Diagram.jpg" width=75%
#
# @section ss_overview Overview
# @subsection ss_tasks Tasks
# Task Name  | Task Function | Task Priority | Task Period [us]
# ------------- | ------------- | ------------- | -------------
# Control  | Controller.Controller.task | 2 | 10
# Tracker  | Tracker.Tracker.task | 1 | 20
# DriveR  | MotorEncoderTask.MotorEncoder.task | 3 | 5
# DriveL  | MotorEncoderTask.MotorEncoder.task| 3 | 5
# \image html "FinalTaskDiagram.jpg" width=50%
#
# @subsection ss_track Track
# \image html "Gametrack Diagram.jpg" width=75%
#
# @subsection ss_bluetooth Bluetooth module
# Romi uses HC-05 Wireless Bluetooth Receiver connected to UART 5. The below code mirrors Micropython's tty to UART 5.
# This allows micropython to communicate over the bluetooth module.
# @code
# # Bluetooth Configuration
# uart = pyb.UART(5, 115200)
# uart.init(115200, bits=8, parity=None, stop=1)
# pyb.repl_uart(uart)
# @endcode
#
# @subsection ss_battery Battery Voltage Adjustor
# Because the motors are effort based instead of voltage based as Romi's batteries drain the motors will behave differently.
# To account for this Romi contains a voltage divider which is used in conjunction with an ADC pin to measure the current battery voltage.
# The following code is located in \ref main.py
# @code
# batPin = ADC(Pin.board.PB0)
# Vbat = batPin.read() * (3.3 / 4095) * (58 / 20) * 1.125
#
# if (Vbat <= 6.65):
#     # If we are low on charge
#     print("CHARGE BATTERIES")
#     exit(0)
# @endcode
#
# The current voltage is then used to calculate motor effort from the desired velocity.
# The following code is located in \ref MotorEncoderTask.py
# @code
# self.motor.set_effort(voltage/Vbat * 100)
# @endcode
# Operating in voltage allows Romi to directly use motor gain for feedforwarding
# @code
# motorGain = 5.57  # (rad/s)/V
# offset = 2.1  # Offset to overcome static friction.
# self.vel2volt = lambda vel: ((vel / motorGain) + offset * (-1 if vel < 0 else 1))
# @endcode

# @section ss_results Results
# @subsection ss_timeTrial Time Trial
# Trial Number | Checkpoint 1 [s] | Checkpoint 2  [s] | Checkpoint 3 [s] | Checkpoint 4 [s] | Checkpoint 5 [s] | Raw Time [s] | Number of Cups | Final Time [s]
# ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
# 1 | 3.06 | 6.03 | 9.20 | 10.83 | 13.07 | 16.90 | 2 | 6.90
# 2 | 3.07 | 6.03 | 9.17 | 10.93 | 13.13 | 16.80 | 2 | 6.80
# 3 | 3.10 | 6.17 | 9.27 | 10.90 | 13.17 | 16.80 | 2 | 6.80