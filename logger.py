from pybricks.tools import DataLog, StopWatch, wait
from lcd_control import LCDControl

class Logger:
    def __init__(self, *colums, name='log', timestamp=False, append=False, ev3=None):
        self.logger = DataLog(colums, name=name, timestamp=timestamp, append=append)
        self.watch = StopWatch()
        self.ev3 = ev3
        lcd_control = LCDControl(brick=ev3)

    def log(self, *data):
        self.logger.log(data)