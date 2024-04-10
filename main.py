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
from lcd_control import LCDControl
from color_control import ColorControl
from state_manager import StateManager
from logger import Logger
from sound_light_control import SoundLightControl
from drive_manager import DriveManager, KP, KI, KD, ERROR_LIMIT

from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.parameters import Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

DEFAULT_SPEED = 100
DEFAULT_TURN_RATE = 0

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
# ev3.speaker.beep()

control = RobotControl()
control.beep(ev3)
distance_control = DistanceControl()
color_control = ColorControl()
lcd_control = LCDControl(brick=ev3)
state_manager = StateManager(brick=ev3)
logger = Logger('distance', 'color', 'timestamp')
sound_light = SoundLightControl(ev3)

left_motor = control.left_motor
right_motor = control.right_motor
# drive_base = DriveManager(left_motor, right_motor)
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)

print(distance_control.distance())
# control.forward(1000)

# forward for 1 minute
now = time.time()
future = now + 10
beep_num = 0
color = Color.BLACK
distance = 0
# control.rotate(-360*2, 500)
# ======================KP======================
errors = []
current_angle = 0
#TP2

while True :#time.time() < future:

    ### TP2
    lcd_control.write('%s : %s' % (color_control.mesure_error(), drive_base.angle()))
    
    errors.append(color_control.mesure_error())
    while KI*sum(errors) >= ERROR_LIMIT:
        errors = errors[100:]
        lcd_control.write('error limit reached')

    if len(errors) > 0:
        current_angle = (KP*color_control.mesure_light()) + KI*sum(errors) + KD*(color_control.mesure_error() - errors[-1])
    else:
        current_angle = (KP*color_control.mesure_light()) + KI*sum(errors)
    
    drive_base.drive(DEFAULT_SPEED, current_angle)
    #logger.log(errors[-1])

    now = time.time()
    color = color_control.mesure_light()
    distance = distance_control.distance()
    lcd_control.write('color: %s' % color)
    logs = [distance, color, now]

    logger.log(*logs)
    

control.beep(ev3)
# control.stop()