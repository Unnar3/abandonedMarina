#!/usr/bin/env python

import numpy as np
from math import sin, cos
#step

class ekf:

    def __init__(self):
        # State is [x,y,z,phi,u,v,w,r,phi_0]
        self.pos = np.matrix([[0],[0],[0],[-2.2],[0],[0],[0],[0]]) 
        self.P = np.eye(self.pos.shape[0])*0.1
        self.lastTime = 0
        self.initialized = False


    def predictmodel(self,T, noise):

        # Use formulas
        self.pos[0] = self.pos[0] + self.pos[4]*T*cos(self.pos[3]) -  self.pos[5]*T*sin(self.pos[3]) 
        self.pos[1] = self.pos[1] + self.pos[4]*T*sin(self.pos[3]) +  self.pos[5]*T*cos(self.pos[3]) 
        self.pos[2] = self.pos[2] + self.pos[6]*T 
        self.pos[3] = self.pos[3] + self.pos[7]*T
        self.pos[4] = self.pos[4]
        self.pos[5] = self.pos[5]
        self.pos[6] = self.pos[6]
        self.pos[7] = self.pos[7]

        # Add noise
        self.pos[0] = self.pos[0] + noise[0]*T**2/2*cos(self.pos[3]) -  noise[1]*T**2/2*sin(self.pos[3])
        self.pos[1] = self.pos[1] + noise[0]*T**2/2*sin(self.pos[3]) +  noise[1]*T**2/2*cos(self.pos[3])
        self.pos[2] = self.pos[2] + noise[2]*T**2/2
        self.pos[3] = self.pos[3] + noise[3]*T**2/2
        self.pos[4] = self.pos[4] + noise[0]*T
        self.pos[5] = self.pos[5] + noise[1]*T
        self.pos[6] = self.pos[6] + noise[2]*T
        self.pos[7] = self.pos[7] + noise[3]*T

    def predictjacobian(self, T, noise):
        phi1 = float(-sin(self.pos[3])*(noise[0]*T**2/2 + T*self.pos[4])-cos(self.pos[3])*(noise[1]*T**2/2 + T*self.pos[5]))
        phi2 = float(cos(self.pos[3])*(noise[0]*T**2/2 + T*self.pos[4])-sin(self.pos[3])*(noise[1]*T**2/2 + T*self.pos[5]))

        M = np.matrix([ [1, 0, 0, phi1, T*cos(self.pos[3]), -T*sin(self.pos[3]), 0, 0],
                        [0, 1, 0, phi2, T*sin(self.pos[3]),  T*cos(self.pos[3]), 0, 0],
                        [0,0,1,0,0,0,T,0],
                        [0,0,0,1,0,0,0,T],
                        [0,0,0,0,1,0,0,0],
                        [0,0,0,0,0,1,0,0],
                        [0,0,0,0,0,0,1,0],
                        [0,0,0,0,0,0,0,0]])
        return M

    
    def predict(self,time):
        
        # Make sure that some values (Time) have been initialized 
        if self.initialized is False:
            self.lastTime = time
            self.initialized = True
            return

        # Find duration in seconds from last prediction
        T = (time - self.lastTime).to_sec()
        self.lastTime = time

        # Create random additive white noise with zero mean
        noise = np.random.normal(0,0.01,4).transpose()
        for i in range(0,len(noise)):
            noise[i] = 0.0

        # Use equations to predict new position
        self.predictmodel(T, noise)
        # Find jacobian of the predictmodel and use it to update the covariance
        F = self.predictjacobian(T, noise)
        self.P = F*self.P*F.T

    def update(self, z, H):
        # print "update"
        if H.shape[0]==1:
            R = 0.01
        else:
            R = np.eye(H.shape[0])*1
        S = H*self.P*H.T + R
        K = self.P*H.T*np.linalg.inv( S )
        self.P = (np.eye(self.P.shape[0]) - K*H)*self.P
        print "K: ", K.T
        print "z: ", z
        print "posB: ", self.pos.T
        print "diff: ", z - H*self.pos
        self.pos = self.pos + K*(z - H*self.pos)
        # print self.P
        print "posA: ", self.pos.T
        print "  "

    def returnState(self):
        x = float(self.pos[0])
        y = float(self.pos[1])
        z = float(self.pos[2])
        theta = float(self.pos[3])
        return x,y,z,theta

# class kalman_filter:
#     #take in all the parameters of the linear kalman filter
#     def __init__(self,A,B,C,Q,P,R,x):
#         self.A = A #state trasition matrix
#         self.B = B#control matrix
#         self.C = C#measurement model
#         self.Q = Q#proccess noise covariance
#         self.P = P#predeciton
#         self.R = R##measurement noise covariance
#         self.state = x
        
#     #move to the  next position
#     def move(self,u,z, R=None):
#         self.predict(u)
#         self.update(z,R)
        
#     #get the current state
#     def getState(self):
#         return self.state
        
#     #predict the next state 
#     def predict(self,u):
#         self.state = self.A*self.state + self.B*u
#         self.P = self.A*self.P*self.A.T + self.Q
    
#     #update the model
#     def update(self,z, R=None):
#         #find the kalman gain
#         if R is None:
#             R = self.R
#         K = self.P*(self.C.T)*np.linalg.inv((self.C*self.P*(self.C.T) +R))
#         #get the current state
#         self.state = self.state + K*(z - self.C*self.state)
#         #get the next prediction
#         size = self.state.shape[0]
#         self.P = (np.eye(size) - K*self.C )*self.P 
        
        
    