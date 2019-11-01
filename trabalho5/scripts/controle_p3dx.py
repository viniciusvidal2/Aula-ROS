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

def calcular_xy(distancia, al):
    global x_r, y_r, t_r

    al = np.pi/180.0*(al - 180) # ajustando o angulo da leitura em questao
    # De acordo com o angulo, calcular x e y no frame do robo a partir da distancia medida
    xfr = distancia*np.cos(al)
    yfr = distancia*np.sin(al)
    # Retornar valores x e y para o ponto de obstaculo - frame robo
    #print('x: {}   y: {}  al: {}'.format(xfr, yfr, al))
    return xfr, yfr

# Definir objetivo com o laser, onde a porta esta
def Laser(data):
    global ja_calc_objetivo
    global x_g, y_g, angulo, G
    global x_r, y_r, t_r

    #Ganhos (campos potenciais)
    Katt = 0.4 # cte de forca atrativa
    Krep = 0.6 # cte de forca repulsiva
    Eps = 0.0 # Distancia do robo ao objetivo
    Ep0 = 4.0 # Cte do horizonte de eventos [m]
    deltaD = 0.01 # Distancia a ser caminhada na direcao
    #ksi = raio da bola de convergencia  
    ksi = 0.2 # Quanto menor ksi -> Mais preciso

    FrepX = 0 # Forca repulsiva inicial
    FrepY = 0
    vmax = 0.2 # Velocidade maxima do robo
    # Constante da velocidade angular
    Kw = 1

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    vel_msg = Twist()

    leituras = np.array(data.ranges)
    if not ja_calc_objetivo:
        if len(leituras) > 3:
            angulos  = np.where(leituras > data.range_max)
            angulos  = angulos - 180*np.ones(len(angulos[0]))
            Indice_distancia_minima = int(angulos[0][ 0] - 1 + 180)
            if Indice_distancia_minima < 0:
                Indice_distancia_minima = 0
                distancia_minima = leituras[Indice_distancia_minima]
            else:
                distancia_minima = leituras[Indice_distancia_minima]
	        # A conta -1 + 180 eh feita para pegar um indice antes daquele em que o valor consta como infinito e o 180 soma para fazer um
            # offset no vetor que varia de -180 a 180 para ele variar de 0 a 360
            Indice_distancia_maxima = int(angulos[0][-1] + 1 + 180)
            if Indice_distancia_maxima > 359:
                Indice_distancia_maxima = 359
                distancia_maxima = leituras[Indice_distancia_maxima]
            else:
                distancia_maxima = leituras[Indice_distancia_maxima]
	        #[-1] se refere ao indice final, equivalente ao end no matlab
            distancia        = (distancia_minima + distancia_maxima)/2

            angulo_minimo = np.min(angulos)
            angulo_maximo = np.max(angulos)
            angulo        = (angulo_minimo + angulo_maximo)/2 * np.pi/180.0
     
   	        # Adaptacao para sinais certos em cada quadrante
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

            G = np.array([[x_g], [y_g]])
            ja_calc_objetivo = True


    #Posicao atual do robo
    Pr = np.array([[x_r], [y_r]])
    #Distancia entre Pr e G
    DPG = Pr - G
    Dpg = np.linalg.norm(DPG)

    if Dpg > ksi and ja_calc_objetivo == True:
        # Calcula a forca de atracao
        Fatt = -Katt*(Pr - G)
        # Definicao das distancias
        Eps = leituras
        # Pontos do obstaculo dentro do horizonte de seguranca
        indices = np.where(Eps < Ep0)
        Indices = indices[0]

        # Verifica se nao ha obstaculos no horizonte de eventos
        if len(Indices) == 0:
            FrepX = 0
            FrepY = 0
        else:
            # Calculo da forca repulsiva
            for k in range(len(Indices)):
                x_obstaculo, y_obstaculo = calcular_xy(leituras[Indices[k]], Indices[k])

                Termo1 = Krep*1/(Eps[Indices[k]]**3) # Adicionei sinal aqui negativo porque aparentemente esta contraria a forca
                Termo2 = (1/Eps[Indices[k]]) - (1/Ep0) # Aqui estava ao contrario
                Termo3X = Pr[0]-x_obstaculo
                Termo3Y = Pr[1]-y_obstaculo
                FrepX = FrepX + Termo1*Termo2*Termo3X
                FrepY = FrepY + Termo1*Termo2*Termo3Y

        # Rotacionar forca para o frame do robo
        FrepX_fm = FrepX*np.cos(t_r) - FrepY*np.sin(t_r)
        FrepY_fm = FrepX*np.sin(t_r) + FrepY*np.cos(t_r)
        # Forca total
        FtotX = Fatt[0] + FrepX_fm
        FtotY = Fatt[1] + FrepY_fm
        #print('FrepX_fm: {}   FrepY_fm: {}'.format(FrepX_fm, FrepY_fm))
        #print('FtotX: {}   FtotY: {}'.format(FtotX, FtotY))
        # Definicao do subobjetivo
        v = np.min(np.array( [np.linalg.norm( np.array([FtotX, FtotY]) ), vmax] ))
        #print(v)
        # Angulo desejado pela forca atrativa
        ang_obj = np.arctan2(Fatt[1], Fatt[0])
        ang = Kw*(np.arctan2(FtotY, FtotX) - t_r)
        print(ang)
        
        vel_msg.linear.x  = v
        vel_msg.linear.y  = 0
        vel_msg.linear.z  = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = ang

        pub.publish(vel_msg)

        rate = rospy.Rate(10)
        rate.sleep()
        #rospy.signal_shutdown('')

    else:
        # Para o robo ao chegar ao objetivo
        vel_msg.linear.x  = 0
        vel_msg.linear.y  = 0
        vel_msg.linear.z  = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
	    
# Ler a odometria e dar o tom a partir dela do que fazer
def Odometria(data):
    global x_r, y_r, t_r

    x_r = data.pose.pose.position.x
    y_r = data.pose.pose.position.y
    t_r = np.arctan2(2*data.pose.pose.orientation.w*data.pose.pose.orientation.z,1-2*data.pose.pose.orientation.z*data.pose.pose.orientation.z)

# Controle de posicao para chegar onde a porta esta
def controle():
    global ja_calc_objetivo
    global G
    ja_calc_objetivo = False

    global x_g, y_g, angulo
    global x_r, y_r, t_r
    x_g = 0
    y_g = 0
    x_r = 0
    y_r = 0   
    t_r = 0
    G = 0
    angulo = 0

    rospy.init_node('controle_p3dx', anonymous=False, disable_signals=True)
    rospy.Subscriber("/scan", LaserScan, Laser    , queue_size=1)
    rospy.Subscriber("/odom", Odometry , Odometria, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    controle()
