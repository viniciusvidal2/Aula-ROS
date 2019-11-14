#!/usr/bin/env python
import rospy
import sys, time
import numpy as np
from math import sqrt, pow, atan2

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

tolerancia = 0.5
toleranciaTh = 0.3

# Definir objetivo com o laser, onde a porta esta
def Laser(data):
    global ja_calc_objetivo
    global x_g, y_g

    if not ja_calc_objetivo:
        leituras = np.array(data.ranges)
        if len(leituras) > 10:
            angulos  = np.where(leituras > data.range_max)
            angulos  = angulos - 180*np.ones(len(angulos))
            print(angulos)

            distancia_minima = leituras[int(angulos[0][ 0] - 1 + 180)]
            distancia_maxima = leituras[int(angulos[0][-1] + 1 + 180)]
            distancia        = (distancia_minima + distancia_maxima)/2

            angulo_minimo = np.min(angulos)
            angulo_maximo = np.max(angulos)
            global angulo
            angulo = (angulo_minimo + angulo_maximo)/2 * np.pi/180.0
     
            if angulo >= 0 and angulo < np.pi/2:                         #    0 <= angulo <  90
                x_g = np.cos(angulo)*distancia #+ 0.85
                y_g = np.sin(angulo)*distancia #- 0.75
            elif angulo >= np.pi/2 and angulo <= np.pi:                  #   90 <= angulo < 180
                x_g = -np.sin(angulo - np.pi/2)*distancia
                y_g =  np.cos(angulo - np.pi/2)*distancia
            elif angulo < -np.pi/2 and angulo >= -np.pi:                 # -180 <= angulo < -90 
                x_g = -np.sin( np.abs(angulo + np.pi/2) )*distancia
                y_g = -np.cos( np.abs(angulo + np.pi/2) )*distancia
            else:                                                        #  -90 <= angulo <   0
                x_g =  np.cos( np.abs(angulo) )*distancia
                y_g = -np.sin( np.abs(angulo) )*distancia

            #rospy.loginfo("Dist minimo: %.1f", distancia_minima)
            #rospy.loginfo("Dist maximo: %.1f", distancia_maxima)

            #rospy.loginfo("Angulo minimo: %.1f", angulo_minimo)
            #rospy.loginfo("Angulo maximo: %.1f", angulo_maximo)
            rospy.loginfo("xg: %.2f   yg: %.2f", x_g, y_g)

            ja_calc_objetivo = True

# Ler a odometria e dar o tom a partir dela do que fazer
def Odometria(data):
    global x_r, y_r, t_r

    x_r = data.pose.pose.position.x
    y_r = data.pose.pose.position.y
    t_r = np.arctan2(2*data.pose.pose.orientation.w*data.pose.pose.orientation.z,1-2*data.pose.pose.orientation.z*data.pose.pose.orientation.z)
    #rospy.loginfo("Xg: %.2f   Yg: %.2f  Xr: %.2f   Yr: %.2f   Tr:%.2f", x_g, y_g, x_r, y_r, t_r)

# Controle de posicao para chegar onde a porta esta
def controle():
    global ja_calc_objetivo
    ja_calc_objetivo = False

    global x_g, y_g, angulo
    global x_r, y_r, t_r
    x_g = 0
    y_g = 0
    x_r = 0
    y_r = 0
    t_r = 0
    angulo = 0

    rospy.init_node('controle_p3dx', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, Laser    , queue_size=1)
    rospy.Subscriber("/odom", Odometry , Odometria, queue_size=1)
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)

    vel_msg = Twist()

    # Ganhos
    Kp = 0.01
    Kh = 2.5

    # Ganhos (campos potenciais)
    #Katt = 0.001 # cte de força atrativa
    #Krep = 0.001 # cte de força repulsiva
    #Ep0 = 3 # Cte do horizonte de eventos [m]
    #deltaD = 0.01 # Distancia a ser caminhada na direção
    #G = [x_g; y_g] # Objetivo
    # ksi = raio da bola de convergência  
    #ksi = 0.1 # Quanto menor ksi -> Mais preciso

    #Frep = [0; 0] # Força repulsiva inicial
    #vmax = 100 # Velocidade máxima do robô
    # Constante da velocidade angular
    #Kw = 0.1
    
    # Posição atual do robô
    #Pr = [x_r; y_r];
    # Distância entre Pr e G
    #DPG = Pr - G;
    #Dpg = (np.transpose(DPG)*DPG)^0.5;

    #while Dpg > ksi
    while not rospy.is_shutdown():
            dx  = x_g - x_r
            dy  = y_g - y_r
            rho = sqrt(dx**2 + dy**2)
            rospy.loginfo("xg: %.2f   yg: %.2f    xr: %.2f   yr: %.2f", x_g, y_g, x_r, y_r)

            #gamma = np.arctan2(dy,dx)
            #alpha = gamma - t_r
            #beta  = th_g - gamma

            alpha = angulo - t_r

            vel_msg.linear.x  = Kp*rho
            vel_msg.linear.y  = 0
            vel_msg.linear.z  = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = Kh*alpha #- 5*beta

            #rospy.loginfo("Velocidade linear: %.2f   Velocidade angular: %.2f", Kp*rho, -Kh*alpha)
            rospy.loginfo("Rho: %.2f", rho)
            #rospy.loginfo("Alpha: %.2f  t_r: %.2f  angulo: %.2f", alpha, t_r, angulo)

            if rho > 1:
                pub.publish(vel_msg)
                rate.sleep()
            else:
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                pub.publish(vel_msg)
                rate.sleep()

if __name__ == '__main__':
    controle()
