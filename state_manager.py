from distance_control import DistanceControl
from robot_control import RobotControl
from color_control import ColorControl

class StateManager():
    def __init__(self):
        self.state = None
        self.distance_control = DistanceControl()
        self.robot_control = RobotControl()
        self.color_control = ColorControl()

    def update_state(self):
        pass