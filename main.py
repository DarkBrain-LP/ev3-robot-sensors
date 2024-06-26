#!/usr/bin/env pybricks-micropython
from umqtt.robust import MQTTClient
 
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
# from Robot import *
# from CapteurDistance import *
from CapteurColor import *
# from Etat import *
# from EcranLCD import *
from pybricks.robotics import DriveBase
import time
import math
from kalman import Kalman
from umqtt.robust import MQTTClient
 
###### Initialisation du robot ######
# par défaut 44 et 114 => correction après test, roues = 56
robot = DriveBase(Motor(Port.B), Motor(Port.C), 56, 114)
# initialisation du capteur couleur
colorSensor = CapteurColor(ColorSensor(Port.S3))
distanceSensor = UltrasonicSensor(Port.S2)
 
# intialisation gyroscope
gyro_sensor = GyroSensor(Port.S4)
MODE_GYRO_ANG = 'GYRO-ANG'
 
###### Initialisation des variables globales ######
error = 0
prevError = 0
sumError = 0
theta = 0
prevdistance = 0
angle_precedent = 0
 
limitSumError = 200
position_PID = (0,0)
 
# cmd bang bang limit
limit = 35
 
# variables init
temps = 0.1
vitesse = 0.1
 
# reset state robot
robot.reset()
print("Robot initialisé")
# init Kalman B Q R P X W
kf = Kalman(0, 1.5*1.5, 1.5*1.5, 1.5*1.5, 0, 0)
 
 
 
# Zone Entrée A : 1
# Zone Entrée B : 2
# Zone de stockage : 3
# Zone de sortie : 4
# Zone de conflit : 5
zone = 0
old_zone = 0
voie = None
 
 
robot_id = 'maestro'
intersection_topic = 'intersection'
intersection_auth_topic = 'intersection/auth'
server_ip = "198.168.119.39"
entree_intersection_topic = 'intersection/request'
sortie_intersection_topic = 'intersection/exit'
intersection_response_topic = 'intersection/response/{}'.format(robot_id)
robot_pub_id = 'robot1'
entree_intersection_message = 'Yu us Here'
sortie_intersection_message = 'robot1;sortie'
 
trasverse_message = 'GRANTED'
 
could_traverse_intersection = True
waiting_cnt = 0
last_waiting_time = 0
waiting_log = DataLog('waiting_time', 'time', name='waiting_time')
# MQTT
client = MQTTClient('yu',server_ip) # 52dc166c-2de7-43c1-88ff-f80211c7a8f6
try:
    print('Connecting to MQTT server')
    client.connect()
    print('Connected to MQTT server')
except Exception as e:
    print('Failed to connect to MQTT server:', str(e))
 
print('Connected to MQTT server')
 
def getmessages(topic, msg):
    global could_traverse_intersection, last_waiting_time
    # check if the message is an intersection
    print('Received message: ', msg, ' on topic: ', topic)
    if topic == intersection_response_topic.encode():
        could_traverse_intersection = True
        print(msg)
        # check if access is granted
        if msg == trasverse_message:
            could_traverse_intersection = True
            waiting_cnt += 1
            # save the time when the robot was granted access
            waiting_log.write('waiting_time', time.time() - last_waiting_time)
            last_waiting_time = time.time()
           
        else:
            could_traverse_intersection = False
        # TODO: get the message and check if we should traverse the intersection
 
client.set_callback(getmessages)
# client.subscribe('JorgePe/test')
 
#client.subscribe(intersection_auth_topic)
client.subscribe(intersection_response_topic)
brick.display.text('Listening...')
 
 
 
 
# calcul PID
# variable kp : corrige les à-coups (force)
# variable ki : corrige la trajectoire si la somme des erreurs dépasse la limite
# variable kd : corrige les oscillations
 
def driveRobot_PID(kp, ki, kd, teta, speed, delay):
    global error, sumError, limit, prevError, limitSumError, limit, position_PID, prevdistance, angle_precedent, zone, old_zone
    global could_traverse_intersection, last_waiting_time
 
    ###### Calcul PID ######
 
    # récupération de la couleur (valeur entre 0 et 100 / noir et blanc)
    reflection = colorSensor.reflection()
    #print("Reflexion: {} ".format(reflection))
 
    # calcul de l'erreur
    error = reflection - limit
    # somme de toutes les erreurs (bornée)
    sumError += error
 
    # si dépassement de la limite de la somme de l'erreur
    # = trajectoire rectifiée
    if prevError * error < 0:
        sumError = 0
 
    if abs(sumError) > limitSumError:
        sumError = limitSumError*(sumError/abs(sumError))
 
    v_angle = error*kp + ki*sumError + (kd)*(error - prevError)
    # le robot avance et varie sa trajectoire en fonction du PID
    robot.drive(speed,v_angle)
 
    # MAJ erreur précédente
    prevError = error
 
 
    ###### FIN Calcul PID ######
 
 
    ##### DEBUT STATE #####
   
    '''
 
    pos_prev_x = position_PID[0]
    pos_prev_y = position_PID[1]
 
    # distance, drive speed, angle and turn rate
    etat = robot.state()
    #print(etat)
   
    theta = etat[2]
    distance = etat[0]
 
    d_distance = distance - prevdistance
    d_theta = theta - angle_precedent
 
    pos_x = math.cos(math.radians(theta + d_theta)) * d_distance + pos_prev_x
    pos_y = math.sin(math.radians(theta + d_theta)) * d_distance + pos_prev_y
 
    angle_precedent = theta
    prevdistance = distance
 
    position_PID = pos_x, pos_y
 
    x = str(position_PID[0]).replace('.', ',')
    y = str(position_PID[1]).replace('.', ',')
    print(x + ";" + y)
 
   
    ##### FIN STATE #####
 
    #### DEBUT GYRO #####
 
    pos_prev_x = position_PID[0]
    pos_prev_y = position_PID[1]
 
    distance = vitesse * temps
    theta = gyro_sensor.angle()
 
 
    pos_x = math.cos(math.radians(theta)) * distance + pos_prev_x
    pos_y = math.sin(math.radians(theta)) * distance + pos_prev_y
 
    position_PID = pos_x, pos_y
 
    x = str(position_PID[0]).replace('.', ',')
    y = str(position_PID[1]).replace('.', ',')
    print(x + ";" + y)
 
    # 100ms de délai entre chaque mesure de couleur
    wait(delay)
 
    #### FIN GYRO #####
 
    '''
   
    ### UTILISATION KALMAN ###
   
 
    pos_prev_x = position_PID[0]
    pos_prev_y = position_PID[1]
 
    # Mesure l'angle avec gyro
    mesure_angle = gyro_sensor.angle()
 
    # Mesure l'angle avec state
    etat = robot.state()
 
    # Filtre de Kalman (mesure = gyro, modèle = state)
    theta_estime = kf.update(mesure_angle,etat[2])
 
    # Calcul de la position
    distance = vitesse * temps
    pos_x = math.cos(math.radians(theta_estime)) * distance + pos_prev_x
    pos_y = math.sin(math.radians(theta_estime)) * distance + pos_prev_y
 
    position_PID = pos_x, pos_y
 
    x = str(position_PID[0]).replace('.', ',')
    y = str(position_PID[1]).replace('.', ',')
    #print(x + ";" + y)
 
    old_zone = zone
   
    if((position_PID[0] > -0.2 and position_PID[0] < 0.4) and (position_PID[1] > -0.2)):
        #print("Je suis sur une zone d'entrée : B")
        zone = 2
 
    if((position_PID[0] > -0.2 and position_PID[0] < 0.4) and (position_PID[1] < -1.6)):
        #print("Je suis sur une zone d'entrée : A")
        zone = 1
 
    if((position_PID[0] < 0.4 and position_PID[0] > 0.2) and (position_PID[1] > -1.6 and position_PID[1] < -0.2) and not (position_PID[1] > -0.9 and position_PID[1] < -0.7)):
        #print("Je suis sur une zone de stockage")
        zone = 3
 
    if((position_PID[0] < 0 and position_PID[0] > -0.2) and (position_PID[1] > -1.6 and position_PID[1] < -0.2) and not(position_PID[1] > -0.9 and position_PID[1] < -0.7)):
        #print("Je suis sur une zone de sortie")
        zone = 4
 
    if((position_PID[0] > 0 and position_PID[0] < 0.2) and (position_PID[1] > -0.9 and position_PID[1] < -0.7)):
        #print("Je suis sur une zone de conflit")
        zone = 5
   
 
    if(old_zone != zone):
        if(zone == 1):
            print("Je suis sur une zone d'entrée : A")
        if(zone == 2):
            print("Je suis sur une zone d'entrée : B")
           
        if(zone == 3):
            print("Je suis sur une zone de stockage")
            voie = "A"
            # publish to the intersection topic
            # TODO: publish to the right message
            entree_intersection_message = robot_pub_id # + ';' + voie
            client.publish(entree_intersection_topic, entree_intersection_message)
           
            could_traverse_intersection = False
            last_waiting_time = time.time()
           
        if(zone == 4):
            print("Je suis sur une zone de sortie")
            # TODO: publish to the right message
            sortie_intersection_message = robot_pub_id #+ ';' + voie
            client.publish(sortie_intersection_topic, sortie_intersection_message)
           
        if(zone == 5):
            print("Je suis sur une zone de conflit")
       
 
 
    wait(delay)
 
 
 
def robotLeader():
    for i in range(100) :
        driveRobot_PID(0.4, 0.13, 0.05, 1.5, 100, 50)
 
    for i in range(100) :
        robot.drive(0,0)
        wait(100)
 
 
 
 
def robotSuiveurToutOuRien():
    #print(distanceSensor.distance())
    etat = robot.state()
    #print(etat[1])
    print(distanceSensor.distance(),";",etat[1])
    if(distanceSensor.distance() < 150):
        robot.drive(0,0)
    else:
        driveRobot_PID(0.4, 0.13, 0.05, 1.5, 150, 100)
 
 
def robotSuiveurA1Point(vitesse):
    #print("distance : " , distanceSensor.distance())
    a = 0.3  # coefficient multiplicateur
    d_t = distanceSensor.distance()  # distance actuelle
    D = 50   # distance de référence
    vitesse_max = 150
 
    vitesse = a * (d_t - D)
    vitesse = max(min(vitesse_max, vitesse), 0)
    #print("vitesse : ", vitesse)
    print(distanceSensor.distance(),";",vitesse)
    driveRobot_PID(0.4, 0.13, 0.05, 1.5, vitesse, vitesse_max)
 
 
def robotSuiveurA2Point(vitesse):
    #print("distance : " , distanceSensor.distance())
 
    a_f = 0.3  # Coefficient multiplicateur pour le freinage
    D_f = 100  # Distance de référence pour le freinage
    a_a = 0.3  # Coefficient multiplicateur pour l'accélération
    D_a = 50  # Distance de référence pour l'accélération
    d_t = distanceSensor.distance()  # distance actuelle
    D = 50   # distance de référence
    vitesse_max = 150
 
    vitesse_freinage = a_f * (d_t - D_f)
    vitesse_freinage = min(max(vitesse_freinage, 0), vitesse_max)
 
    vitesse_acceleration = a_a * (d_t - D_a)
    vitesse_acceleration = min(max(vitesse_acceleration, 0, vitesse), vitesse_max)
 
    vitesse =  min(vitesse_freinage, vitesse_acceleration)
    print(distanceSensor.distance(),";",vitesse)
    driveRobot_PID(0.4, 0.13, 0.05, 1.5, vitesse, vitesse_max)
 
 
 
# Boucle infinie
 
 
# main, boucle infinie
 
while(1):
    while not could_traverse_intersection:
        client.check_msg()
            
    robotSuiveurA2Point(150)
       
#    time.sleep(0.1)
   
    # ligne droite (a été fait avec tétà = 0.1)
    # driveRobot_PID(0.4, 0.13, 0.05, 1.5, 100, 100)
 
    # circuit 8
    #driveRobot_PID(0.4, 0.13, 0.05, 1.5, 100, 100)
 
    #robotLeader()
    #robotSuiveurToutOuRien()
    #robotSuiveurA1Point(150)
    # circuit vague (a été fait avec tétà = 0.1)
    # wait de 80
    #driveRobot_PID(1.8, 2, 0.005, 0.1, 100, 80)
    # wait de 100
    #driveRobot_PID(1.45, 1.1, 0.005, 0.1, 80, 100)