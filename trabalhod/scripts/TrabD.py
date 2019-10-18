#!/usr/bin/env python
import rospy
import sys, time
from std_msgs.msg import String

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pow, atan2
import numpy as np

x_g  = 0
y_g  = 0
th_g = 0

nome = "turtle1"

tolerancia = 0.5
toleranciaTh = 0.3

def callback(data):
    x_r  = data.x
    y_r  = data.y
    th_r = data.theta

    vel_msg = Twist()

    dx  = x_g - x_r
    dy  = y_g - y_r
    rho = sqrt(pow(dx,2)+pow(dy,2))

#    rospy.loginfo("rho: %f, x_r: %f, y_r: %f", rho, x_r, y_r)

    gamma = np.arctan2(dy,dx)
    alpha = gamma - th_r
    beta  = th_g - gamma
    rospy.loginfo("gamma: %f, alpha: %f, beta: %f", gamma, alpha, beta)

    vel_msg.linear.x  = 0.3*rho
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 2*alpha - 5*beta

    #rospy.loginfo("w: %f, gamma: %f, alpha: %f, beta: %f, th_r: %f, th_g: %f", vel_msg.angular.z, gamma, alpha, beta, th_r, th_g)
#    rospy.loginfo("x_g: %f, y_g: %f, th_g: %f", x_g, y_g, th_g)

    global nome
    pub = rospy.Publisher("/"+nome+"/cmd_vel", Twist, queue_size=1)

    if rho > tolerancia or abs(th_g - th_r) > toleranciaTh:
        pub.publish(vel_msg)

    
def RodaNo(x, y, t, n):
    
    rospy.init_node('TrabD', anonymous=True)

    global x_g, y_g, th_g, nome
    x_g = x
    y_g  = y
    th_g = t
    nome = n

    rospy.Subscriber("/"+nome+"/pose", Pose, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("usage: x y theta nome")
    else:
        RodaNo(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
