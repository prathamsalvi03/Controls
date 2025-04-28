import numpy as np


def eq_cart(x,t,u):
    m = 2;
    M=6;
    L =4;
    g =9.81;
    d =1;

    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = m*L*L*(M+m*(1-Cx**2))


    dxdt = [x[1],
            (1/D)*(m**2*L**2*g*Cx*Sx + m*L**2*(m*L*x[3]**2*Sx - d*x[1])) + m*L*L*(1/D)*u,
            x[3],
            (1/D)*(-(m+M)*m*g*L*Sx - m*L*Cx*(m*L*x[3]**2*Sx - d*x[1])) - m*L*Cx*(1/D)*u]

            
            
    return dxdt
