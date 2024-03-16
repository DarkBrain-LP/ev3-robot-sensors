from pybricks.tools import DataLog, StopWatch, wait

class Logger:
    def __init__(self, *colums, name='log', timestamp=True, append=False):
        self.logger = DataLog(columns=colums, name=name, timestamp=timestamp, append=append)
        self.watch = StopWatch()

    def log(self, *data):
        self.logger.log(data)