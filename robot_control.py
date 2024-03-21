from pybricks.ev3devices import Motor
from pybricks.parameters import Port

class RobotControl:
    def __init__(self, left_port=Port.B, right_port=Port.C):
        self.left_motor = Motor(left_port)
        self.right_motor = Motor(right_port)

    def beep(self, ev3):
        ev3.speaker.beep()

    def forward(self, speed):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
    
    def rotate(self, angle, speed):
        if angle > 0:
            self.left_motor.run_angle(speed, angle)
        elif angle < 0:
            self.right_motor.run_angle(speed, abs(angle))
    
    def rotate_backward(self, angle, speed):
        if angle > 0:
            self.right_motor.run_angle(-speed, angle)
        elif angle < 0:
            self.left_motor.run_angle(-speed, abs(angle))

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
