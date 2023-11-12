# -*- coding: utf-8 -*-

import casadi as cas
import system_params
import numpy as np


m,g,m1,m2,l1,l2 = system_params.get_params()

# friction
d1 = 0.0
d2 = 0.0
d3 = 0.0

def dynamic_cart(t, x, Tend, u=0):
    #x = [ksi, theta1, theta2, ksi_dot, theta1_dot, theta2_dot]
    
    M = cas.MX.zeros(3,3)
    f = cas.MX.zeros(3)
    dx = cas.MX.zeros(6)
    
    M[0,0] = m + m1 +m2
    M[0,1] = l1*(m1+m2)*cas.cos(x[1])
    M[0,2] = m2*l2*cas.cos(x[2])
    
    M[1,0] = l1*(m1+m2)*cas.cos(x[1])
    M[1,1] = l1**2*(m1+m2)
    M[1,2] = l1*l2*m2*cas.cos(x[1]-x[2])
    
    M[2,0] = l2*m2*cas.cos(x[2])
    M[2,1] = l1*l2*m2*cas.cos(x[1]-x[2])
    M[2,2] = l2**2*m2
    
    f[0] = l1*(m1+m2)*x[4]**2*cas.sin(x[1])+m2*l2*x[5]**2*cas.sin(x[2]) + u -d1*x[3]
    f[1] = -l1*l2*m2*x[5]**2*cas.sin(x[1]-x[2])+g*(m1+m2)*l1*cas.sin(x[1])  -d2*x[4]
    f[2] = l1*l2*m2*x[4]**2*cas.sin(x[1]-x[2]) + g*l2*m2*cas.sin(x[2]) -d3*x[5]
    
    dy = cas.inv(M) @ f
        
    dx[0] = x[3]
    dx[1] = x[4]
    dx[2] = x[5]
    dx[3] = dy[0]
    dx[4] = dy[1]
    dx[5] = dy[2]
    

    dx = dx 
    
    return dx