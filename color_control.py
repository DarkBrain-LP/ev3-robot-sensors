from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port

class ColorControl:
    def __init__(self, port=Port.S3):
        self.color_sensor = ColorSensor(port)

    def color(self):
        return self.color_sensor.color()

    def reflect(self):
        return self.color_sensor.reflection()

    def ambient(self):