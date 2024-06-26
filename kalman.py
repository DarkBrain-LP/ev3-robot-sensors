import math

class Kalman():
    # KALMAN filter implementation
    def __init__(self):
        self.X0 = 0
        self.P0 = 1.5
        self.B = 5
        self.Un = 0 # pid command
        self.Rn = math.radians(10) # process noise
        self.Wn = self.Rn
        self.Qn = 5 #self.Rn**2
        self.Zn = 0 # gyro mesurement

        self.Xnm = 0 #self.X0 + self.B*self.Un + self.Wn # estimated position
        self.Pnm = 1.5 #self.P0 + self.Qn
        self.Kn = 0 #self.Pnm / (self.Pnm + self.Rn)
        self.Yn = 0 #self.Zn - self.Xnm
        # self.Xnm = self.Xnm + self.Kn*self.Yn
        # self.Pnm = (1-self.Kn) * self.Pnm

        self.time = 0
        self.Xn = 0
        self.Pn = 1.5
        self.A = 1

    def filter(self, mesured_angle, model_angle):
        self.Un = model_angle
        self.Zn = mesured_angle

        self.Xnm = self.Xnm + self.B*self.Un + self.Wn # estimated position
        self.Pnm = self.Pnm + self.Qn
        self.Pnm = self.A * self.Pnm * self.A + self.Qn

        
        self.Kn = self.Pnm / (self.Pnm + self.Rn)
        # self.Kn = 1.5 / (1.5 + self.Rn)
        self.Yn = self.Zn - self.Xnm
        self.Xnm = self.Xnm + self.Kn*self.Yn
        self.Pnm = (1-self.Kn) * self.Pnm
        
        return self.Xnm
    
        self.Xnm = self.Xn + self.time*model_angle
        self.Pnm = self.Pn + self.Qn
        self.Kn = self.Pnm / (self.Pnm + self.Rn)
        self.Yn = self.Zn - self.Xnm
        self.Xn = self.Xnm + self.Kn*self.Yn


