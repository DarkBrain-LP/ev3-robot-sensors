
# KP = 0.9#1.7
# KI = 0.3#0.15
# KD = 0.01#0.1
# KP = 0.9
# KI = 0.4
# KD = 0.01
# KP = 0.9
# KI = 0.07
# KD = 0.008
# KP = 0.6
# KI = 0.3
# KD = 0.02
# KP = 1.7
# KI = 0.15
# KD = 0.1
# x2
# KP = 1.5
# KI = 0.9
# KD = 0.207
from pybricks.robotics import DriveBase

KP = 1.7
KI = 0.15
KD = 0.1

DEFAULT_SPEED = 100
DEFAULT_TURN_RATE = 0
ERROR_LIMIT = 10
LEFT_ANGLE = 20
RIGHT_ANGLE = -20
# C droite B gauche couleur 3 distance 2

class DriveManager:
    def __init__(self, left_motor, right_motor, wheel_diameter=56, axle_track=112, color_control=None):
        self.drivebase = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)
        self.color_control = color_control
    
    def drive(self, speed=DEFAULT_SPEED, turn_rate=DEFAULT_TURN_RATE):
        self.drivebase.drive(speed, turn_rate)

    def bang_bang_kp(self, error, kp=1):
        self.drivebase.drive(error*kp, 0)

    def drive_bang_bang():
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, LEFT_ANGLE)
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, RIGHT_ANGLE)

    def drive_kp():
        current_angle = abs(KP*self.color_control.mesure_light())
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, current_angle) 
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, -current_angle) 

    def drive_pi():
        current_angle = abs(KP*self.color_control.mesure_light())
        errors.append(color_control.mesure_error())
        while KI*sum(errors) >= ERROR_LIMIT:
            errors = errors[100:]
            lcd_control.write('error limit reached')

        urrent_angle = (KP*color_control.mesure_light()) + KI*sum(errors)
        
        drive_base.drive(DEFAULT_SPEED, current_angle)
        logger.log(errors[-1])

    def drive_pid():
        current_angle = abs(KP*self.color_control.mesure_light())
        errors.append(color_control.mesure_error())
        while KI*sum(errors) >= ERROR_LIMIT:
            errors = errors[100:]
            lcd_control.write('error limit reached')

        if len(errors) > 0:
            current_angle = (KP*color_control.mesure_light()) + KI*sum(errors) + KD*(color_control.mesure_error() - errors[-1])
        else:
            current_angle = (KP*color_control.mesure_light()) + KI*sum(errors)
        
        drive_base.drive(DEFAULT_SPEED, current_angle)
        logger.log(errors[-1])

    def turn(self):
        angle = self.drivebase.angle()

        return self.drivebase.turn(angle)
    
    def turn_left(self, angle):
        pass
