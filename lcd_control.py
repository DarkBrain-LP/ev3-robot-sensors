
class LCDControl:
    def __init__(self):
        self.lcd = LCD()

    def display(self, message):
        self.lcd.display(message)