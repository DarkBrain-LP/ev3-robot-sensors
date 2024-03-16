from pybricks.parameters import Color

class SoundLightControl:
    def __init__(self, ev3, color=Color.RED):
        self.sound = ev3.speaker
        self.light = ev3.light

    def blink(self, color=Color.RED, duration=1000):
        self.light.on(color)
        self.sound.beep()
        self.light.off()
        self.sound.beep()
        self.light.off()
        self.sound.beep()
