import numpy as np
from scipy.integrate  import odeint
import matplotlib.pyplot as plt
from Equation_cartpole import eq_cart
import control as ct

m = 2;
M=6;
L =4;
g =9.81;
d =1;


x0 = np.array([0,0,np.pi+np.pi/8 , 0])
dt = 0.01


duration = 10
totalSteps = np.floor(duration/dt)
t = np.arange(0,duration , dt)

x        = [x0[0]]  
xDot     = [x0[1]]
theta    = [x0[2]]
thetaDot = [x0[3]]

force = [0]

ref = np.array([1,0,np.pi ,0])


#  for theta = pi also when linearizing you ignore second order terms like w^2

A = np.array([[0, 1, 0, 0],
               [0, -d/M, -m*g/M, 0], 
               [0, 0, 0, 1], 
               [0, -d/(M*L), (m+M)*g/(M*L), 0]])

B = np.array([0, 1/M, 0, 1/(M*L)]).reshape((4,1))


# penalty matrices 
Q = np.diag([1.1,0.9,1.5,0.9])
R = 0.001

K = ct.lqr(A,B,Q,R)[0]


for i in  range(1, int(totalSteps)):
    print(f'Step {int(i)}/{int(totalSteps)} \n') # Print current step
    
    u = -K @ (x0-ref)

    force.append(np.array(u[0]))

#integrate from 0 to dt only 1 timestep with 2 points , should take input as x0 ,t,u and return state vector

    states = odeint(eq_cart , x0 , np.linspace(0,dt,2) , args = (u[0],))  

    x0 = states[-1]

    # Output States
    x.append(x0[0])
    xDot.append(x0[1])
    theta.append(x0[2])
    thetaDot.append(x0[3])


plt.figure(1)

plt.subplot(2, 2, 1)
plt.plot(t, ref[0]*np.ones([len(t),1]), '--k', t, x, 'b')
plt.ylabel('Cart Position, x (m)')
plt.legend(['Reference', 'x position'])
plt.xlim(0,duration)

plt.subplot(2, 2, 2)
plt.plot(t, xDot, 'b')
plt.ylabel('Cart Velocity, x'' (m/s)')
plt.xlim(0,duration)

plt.subplot(2, 2, 3)
plt.plot(t, np.rad2deg(ref[2])*np.ones([len(t),1]), '--k', t, np.rad2deg(theta), 'g')
plt.ylabel('Pendulum Position, theta (deg)')
plt.legend(['Reference', 'Angle θ'])
plt.xlim(0,duration)

plt.subplot(2, 2, 4)
plt.plot(t, np.rad2deg(thetaDot), 'g')
plt.ylabel('Pendulum Angular Velocity, thetaDot (deg/s)')
plt.xlim(0,duration)

plt.xlabel('Time, t (s)')


# Control Input (Force)
plt.figure(2)
plt.plot(t, force, 'm')
plt.ylabel('Control Input (Force), F (N)')
plt.xlabel('Time, t (s)')
plt.xlim(0,10)

plt.show() # Show figures