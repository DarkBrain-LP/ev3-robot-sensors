from pybricks.parameters import Color

class StateManager():
    def __init__(self, brick, distance=0, color=None):
        brick.light.on(Color.WHITE)
        self.ev3 = brick
        self.state = None
        self.distance = distance
        self.color = color

    def update_state(self, distance, color):
        self.distance = distance
        self.color = color
    
    def update_distance(self, distance):
        self.distance = distance
    
    def update_color(self, color):
        self.color = color

    def get_state(self)->dict:
        '''return the current state as a dictionary'''
        return {'distance': self.distance, 'color': self.color}