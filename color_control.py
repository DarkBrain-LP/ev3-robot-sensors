from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
from pybricks.parameters import Color

class ColorControl:
    def __init__(self, port=Port.S3, error=20):
        self.color_sensor = ColorSensor(port)
        self.error = error

    def rgb(self):
        return self.color_sensor.rgb()
    def color(self):
        return self.color_sensor.color()

    def reflect(self):
        return self.color_sensor.reflection()

    def ambient(self):
        return self.color_sensor.ambient()

    def is_color(self, color: Color):
        return self.color() == color
    
    def is_intersected(self):
        return -10 <= self.mesure_light() <= 10

    def is_following_black_line(self):
        # return is_color(Color.WHITE)
        # return self.reflect() - self.error >= 0
        # return 0 < self.reflect() < 25
        return self.mesure_light() < 0

    def is_following_white_line(self):
        # return self.reflect() + self.error >= 100
        # return 60 < self.reflect() < 70
        return self.mesure_light() > 0
        
    def mesure_light(self):
        return self.reflect()-self.error
    
    def mesure_error(self):
        return self.reflect()-self.error

    def is_rgb(self, rgb: tuple):
        return self.rgb() == rgb
    
    def is_rgb_similar(self, rgb: tuple, tolerance: int):
        return abs(self.rgb()[0] - rgb[0]) < tolerance and abs(self.rgb()[1] - rgb[1]) < tolerance and abs(self.rgb()[2] - rgb[2]) < tolerance

    def get_rgb_mean(self):
        return (self.rgb()[0] + self.rgb()[1] + self.rgb()[2]) / 3

    
    