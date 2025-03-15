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
# \image html Romi.png width=75%
#
# This section serves to break down each hardware component individually and provides a brief description
# of the purpose and implementation of each.
#
# @subsection ss_wiring_diagram Wiring Diagram
# A comprehensive wiring diagram is provided below for reference throughout the section.
# \image html "Final Wiring Diagram.jpg" width=50%
#
# @subsection ss_nucleo Nucleo MCU Development Board & “Shoe of Brian”
#
# The Nucleo-L476RG is a development board from STMicroelectronics featuring the STM32L476RG microcontroller, a
# single-core microcontroller with 1 MB of flash memory and 128 KB of RAM. The Nucleo features two, 38-pin male
# headers that provide access to PWM timer channels, ADC inputs, I2C communication and more. The team maximized the
# capacity of the board, ultimately being bottlenecked by the available ADC input pins. The board provided an
# excellent base for the team to develop embedded programming skills in Micropython, skills which are easily
# transferrable to other STM32-based microcontrollers, an increasingly common platform.
#
# The Shoe-of-Brian is a custom auxiliary PCB attached directly to the
# headers on the back side of the Nucleo, and functions primarily to provide direct UART
# communication access to the MCU, bypassing the included “ST-Link” attached to the Nucleo. This allowed teams to
# flash Micropython code directly to the MCU through a mini-USB port on the Shoe.
#
# @subsection ss_bluetooth Bluetooth module
# A shortcoming of the Nucleo-L476RG that the team identified was the absence of on-board Bluetooth communication.
# The team knew that it would be invaluable to use serial communication for debugging and performance analysis.
# Doing so without Bluetooth required leaving Romi plugged in via mini-USB while running, greatly slowing down
# development. Accordingly, the team implemented an HC-05 wireless Bluetooth receiver connected to UART 5.
# The code below mirrors Micropython's tty (text-only terminals) to UART 5. This allows the MCU to communicate
# serial data over Bluetooth.
#
# @code
# # Bluetooth Configuration
# uart = pyb.UART(5, 115200)
# uart.init(115200, bits=8, parity=None, stop=1)
# pyb.repl_uart(uart)
# @endcode
#
# @subsection ss_motor_driver Motor Driver and Power Distribution Board
#
# The Motor Driver and Power Distribution Board is designed specifically for Romi’s chassis by
# Pololu Robotics. It features dual motor drivers for independently controlling the two brushed DC
# motors included on Romi, along with convenient connections for the magnetic quadrature encoders attached to
# Romi’s motors. Additionally, it provides onboard power regulation to distribute power from six AA batteries
# at various voltages from logic levels, 3.3V and 5V, to full battery power.
#
# The team utilized all power levels provided by the board. 3.3V for the MCU, 5V for the servo,
# and unregulated battery power for the motors. The specifications of the board were frequented to assure proper
# current could be provided through the regulator to all components.
#
# @subsection ss_motor_encoder Motors and Encoders
#
# Romi features two plastic gearmotors with a reduction gearing 120:1. The motors have extended backshafts to
# allow 12 CPR, magnetic quadrature encoders to attach directly. The motors and encoders are attached directly
# to the PBD as detailed on the Polulu website and are thus omitted from the wiring diagram.
# The motors were experimentally characterized to calculate their time constant and gain value based
# on the following equation.
#
# [EQN]
#
# To achieve this, the team ran Romi at varying input voltages and recorded velocity readings in
# radians per second. The resulting graphs display import motor characteristics which were used to allow
# the team to instruct specific motor velocities during the time trials.
#
# [GRAPHS]
#
# @subsection ss_linesensor  IR Reflectance Sensor Array (“Line Sensor”)
# Line detection was one of the primary ways to navigate the track quickly. For that, the team opted to use an array
# of 13 infrared reflectance sensors, which emit light from a small diode and output a voltage depending on how
# reflective the surface is that they are facing. After researching the availability, the team made a pivotal trade-off decision.
#
# These sensors are available in both digital and analog output formats, an each presented challenges.
# Digital reflectance sensors require a GPIO pin to first be set as an output high, then as an input.
# The reflectance reading can then be calculated from the time it takes for the signal to decay.
# Analog sensors are much quicker by comparison, but the Nucleo is far shorter on analog inputs than digital.
# Additionally, the arrays are available in two different pitches, 8mm and 4mm. A tighter pitch lessens the
# tracking width, but increases the resolution of the measurement, allowing for detection of different line thicknesses.
#
# Ultimately, the team decided to choose an analog sensor with a 4mm pitch, which utilized as many unallocated
# analog channels as possible with 13. The digital sensor required far too much time to be polled fast enough
# to navigate the course at high speeds, and the narrow pitch was an integral element in the teams strategy to
# track the certain landmarks based on line width.
#
# To retrieve a line position reading from the sensor, the equation to calculate the x-position of a 2D shape’s
# centroid was adopted to an array of line sensor readings and their corresponding positions.
#
# \image html Sumval.png width=30%
# \image html centroid.png width=40%
#
# Via this equation, the team was able to achieve a reading for the line position relative to the sensor position.
# This was subtracted from the middle position, sensor 7, to receive a difference used to control Romi.
# This is further discussed in the Software section and can be explored in the associated code documentation.
#
# @subsection ss_imu IMU
# For sections of the course where lines were not available to follow, the team needed a way to control both the
# forward velocity and orientation of Romi. The encoders provided sufficient information to ensure specific
# forward velocity, as well as track the distance Romi has traveled, but do not provide a reliable way to determine
# orientation. To fulfil this requirement, the team decided to use an IMU, the BNO055 chip from Bosch, attached to
# a breakout board from Adafruit which provided the necessary circuity and through-hole access to the I2C pins.
#
# Implementation of the IMU required thorough and diligent examination of the associated documentation.
# This allowed the team to calibrate each of the three individual sensors—magnetometer, accelerometer, and
# gyroscope—and access the calibration coefficients for storage so that they could be manually written to the
# device when power-cycled to avoid repeated calibration. Additionally, the documentation provided reference to
# the device and register addresses necessary for writing to and reading data from the IMU over
# I2C communication protocol.
#
# Heading data was originally taken from the magnetometer, however, the team encountered reliability
# difficulties and instead switched to receiving heading data primarily from the fused gyroscope data
# integrated into the IMU. This heading data was used similarly to the line position data, as controller
# feedback to ensure Romi could drive at a consistent heading.
#
# @subsection ss_bump Bump Sensors
#
# Switch-style bump sensors were used to fulfil the wall-detection requirement of the track, as well as provide the
# unexpected benefit of an emergency shutoff switch. Originally, Romi featured a set of two, three-switch bump
# sensors from Pololu, offering protection for the entire front half of Romi. Eventually, the servo displaced
# the right bump sensor, as it was unnecessary for the requirements of the challenge.
#
# The bump sensor implementation is perhaps the simplest of the whole design, and only requires each switch be
# wired to a GPIO port. Each pin is then assigned to the same interrupt callback in the program, as it is not
# required to determine which of the three switches was activated. The interrupt can contain any functionality
# needed depending on the section of the track Romi is in.
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
# \image html "Gametrack Diagram.jpg" width=50%
# @section ss_results Results
# @subsection ss_timeTrial Time Trial
# Trial Number | Checkpoint 1 [s] | Checkpoint 2  [s] | Checkpoint 3 [s] | Checkpoint 4 [s] | Checkpoint 5 [s] | Raw Time [s] | Number of Cups | Final Time [s]
# ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
# 1 | 3.06 | 6.03 | 9.20 | 10.83 | 13.07 | 16.90 | 2 | 6.90
# 2 | 3.07 | 6.03 | 9.17 | 10.93 | 13.13 | 16.80 | 2 | 6.80
# 3 | 3.10 | 6.17 | 9.27 | 10.90 | 13.17 | 16.80 | 2 | 6.80