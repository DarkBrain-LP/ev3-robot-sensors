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
angle = 0
LEFT_ANGLE = 10
RIGHT_ANGLE = -10
# control.rotate(-360*2, 500)
# ======================KP======================
errors = []
current_angle = 0
#TP2

while True :#time.time() < future:
    '''
    TP1
    
    now = time.time()
    if now < future:
        # display the current status
        lcd_control.status(state_manager.get_state())
        future = now + 10
        logger.log(*state_manager.get_state().values()) #, now

    color = color_control.color()
    state_manager.update_color(color)
    # lcd_control.write(color)
    if color == Color.RED:
        wait(1000)
        control.beep(ev3)
        control.beep(ev3)
        control.beep(ev3)

    if(distance_control.presence()):
        distance = distance_control.distance()
        state_manager.update_distance(distance)
        control.stop()
        control.beep(ev3)
        beep_num += 1
        wait(1000)

        if distance_control.presence():
            control.rotate_backward(360, 100)
            if distance_control.presence():
                control.rotate_backward(-360, 100)
        continue

    control.forward(500)
    sound_light.blink()
    '''

    ### TP2
    lcd_control.write('%s : %s' % (color_control.mesure_error(), drive_base.angle()))
    errors.append(color_control.mesure_error())
    while KI*sum(errors) >= ERROR_LIMIT:
        errors = errors[100:]
        lcd_control.write('error limit reached')

    current_angle = (KP*color_control.mesure_light()) + KI*sum(errors) + KD*(color_control.mesure_error() - errors[-1])
    # now = time.time()
    # if now > future:
    #     if angle == 0:
    #         angle = 45
    #     else:
    #         angle = 0
    #     future = now + 10
    #     drive_base.drive(DEFAULT_SPEED, angle)
    
    # if color_control.is_fo t_angle) #RIGHT_ANGLE
    
    drive_base.drive(DEFAULT_SPEED, current_angle)
    # else:
    #     drive_base.drive(DEFAULT_SPEED, 0)

    # wait(100)
    # drive_base.drive(DEFAULT_SPEED, -angle)
    # drive_base.stop()
    
    # mesured_color = color_control.mesure_error()
    # if mesured_color < 0: # black
    #     pass
    #     #turn left
    # else:
    #     pass
    #     #turn right
    # if not color_control.is_intersected():
    #     if color_control.is_following_black_line():
    #         drive_base.drive(DEFAULT_SPEED, -angle)
    #         # drive_base.drive(turn_rate=45)
    #     elif color_control.is_following_white_line():
    #         drive_base.drive(DEFAULT_SPEED, angle)
    # else:
    #     drive_base.drive(DEFAULT_SPEED, angle)
    
    # wait(100)
    


        
# control.rotate(90, 100)
# control.stop()
control.beep(ev3)
# control.stop()