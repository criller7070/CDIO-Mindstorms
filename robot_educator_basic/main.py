#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Set volume to maximum (0-100)
ev3.speaker.set_volume(100)

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Set speed for all movements (in mm/s)
robot.settings(straight_speed=999999999, turn_rate=999999999)

# Go forward and backwards for one meter.
#robot.straight(1000)

ev3.speaker.beep()
#ev3.speaker.play_file("/home/robot/CDIO-Mindstorms/gyal.mp3")

ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.say("tight pussy girl")
ev3.speaker.say("i like ma sootcase")



robot.straight(-1000)
ev3.speaker.beep()

# Turn clockwise by 360 degrees and back again.
robot.turn(360)
ev3.speaker.beep()

robot.turn(-360)
ev3.speaker.beep()
