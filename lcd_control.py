
class LCDControl:
    def __init__(self, brick):
        self.lcd = brick.screen

    def display(self, message):
        self.lcd.print(message)
    
    def write(self, message):
        self.lcd.print(message)
    
    def status(self, message:dict):
        # iterate through the dictionary and print the key value pairs
        for key, value in message.items():
            self.display(f'{key}: {value}\n')