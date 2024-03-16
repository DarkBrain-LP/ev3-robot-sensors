from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
from pybricks.parameters import Color

class ColorControl:
    def __init__(self, port=Port.S3):
        self.color_sensor = ColorSensor(port)

    def is_color(self, color: Color):
        return self.color() == color

    def is_rgb(self, rgb: tuple):
        return self.rgb() == rgb
    
    def is_rgb_similar(self, rgb: tuple, tolerance: int):
        return abs(self.rgb()[0] - rgb[0]) < tolerance and abs(self.rgb()[1] - rgb[1]) < tolerance and abs(self.rgb()[2] - rgb[2]) < tolerance

    def get_rgb_mean(self):
        return (self.rgb()[0] + self.rgb()[1] + self.rgb()[2]) / 3

    def rgb(self):
        return self.color_sensor.rgb()
    def color(self):
        return self.color_sensor.color()

    def reflect(self):
        return self.color_sensor.reflection()

    def ambient(self):
        return self.color_sensor.ambient()
    