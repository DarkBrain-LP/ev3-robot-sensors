from pybricks.robotics import DriveBase
DEFAULT_SPEED = 75
DEFAULT_TURN_RATE = 0
class DriveManager:
    def __init__(self, left_motor, right_motor, wheel_diameter=56, axle_track=112):
        self.drivebase = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)
    
    def drive(self, speed=DEFAULT_SPEED, turn_rate=DEFAULT_TURN_RATE):
        self.drivebase.drive(speed, turn_rate)

    def turn(self):
        angle = self.drivebase.angle()

        return self.drivebase.turn(angle)
    
    def turn_left(self, angle):
        pass
