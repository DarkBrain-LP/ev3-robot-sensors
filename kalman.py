#!/usr/bin/env pybricks-micropython
import sys
import time
import math

class Kalman(object):
    def __init__(self, B, Q, R, P, X, W):
        self.B = B
        self.Q = Q
        self.R = R
        self.P = P
        self.X = X
        self.W = W
    
    def update(self, angle_mesure, angle_modele):
        # Prédiction

        #X_pred = self.X + self.B * angle_modele + self.W
        X_pred = angle_modele
        P_pred = self.P + self.Q
            
        # Calcul du gain de Kalman
        K = P_pred / (P_pred + self.R)
            
        # Mise à jour avec la mesure
        self.X = X_pred + K * (angle_mesure - X_pred)
        self.P = (1 - K) * P_pred
        
        return self.X



        
