
class LCDControl:
    def __init__(self, brick=None):
        self.lcd = brick.screen if brick is not None else None

    def display(self, message):
        if self.lcd is not None:
            self.lcd.print(message)
    
    def write(self, message):
        if self.lcd is not None:
            self.lcd.print(message)
    
    def status(self, message:dict):
        # iterate through the dictionary and print the key value pairs
        if self.lcd != None:
            for key, value in message.items():
                self.display('%s : %s' % (key, value))
        