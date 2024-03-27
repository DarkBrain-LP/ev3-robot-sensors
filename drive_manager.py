from pybricks.robotics import DriveBase
DEFAULT_SPEED = 75
DEFAULT_TURN_RATE = 0
KP = 1.5
LEFT_ANGLE = 10
RIGHT_ANGLE = -10

color_control = None
drive_base = None
class DriveManager:
    def __init__(self, left_motor, right_motor, wheel_diameter=56, axle_track=112, color_control=None):
        self.drivebase = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)
        self.color_control = color_control
    
    def drive(self, speed=DEFAULT_SPEED, turn_rate=DEFAULT_TURN_RATE):
        self.drivebase.drive(speed, turn_rate)

    def bang_bang_kp(self, error, kp=1):
        self.drivebase.drive(error*kp, 0)

    def a_1():
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, LEFT_ANGLE)
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, RIGHT_ANGLE)
    def a_2():
        color_control = None
        current_angle = abs(KP*self.color_control.mesure_light())
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, current_angle) #LEFT_ANGLE
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, -current_angle) #RIGHT_ANGLE
    def turn(self):
        angle = self.drivebase.angle()

        return self.drivebase.turn(angle)
    
    def turn_left(self, angle):
        pass
