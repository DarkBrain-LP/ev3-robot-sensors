#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from robot_control import RobotControl
from distance_control import DistanceControl

from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
# ev3.speaker.beep()

control = RobotControl()
control.beep(ev3)
distance_control = DistanceControl()
print(distance_control.distance())
print('===========')
print('===========')
print('===========')
print('===========')
# control.forward(1000)

# forward for 1 minute
now = time.time()
future = now + 10
beep_num = 0
while True :#time.time() < future:
    now = time.time()
    control.rotate(-90, 1000)
    # control.rotate(-90, 100)
    control.beep(ev3)
    # control.forward(100)
    # if(distance_control.presence()):
    #     control.stop()
    #     control.beep(ev3)
        # beep_num += 1
        
# control.rotate(90, 100)
# wait(2000)
# control.stop()
control.beep(ev3)
# control.stop()