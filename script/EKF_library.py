import numpy as np
from numpy import dot, zeros, eye
from numpy.linalg import inv
import math 

class KF:
    def __init__(self):

        delta_t=0
        # Transition matrix
        self.dim_x = 4
        self.dim_z = 2
        self.x = zeros((self.dim_x, 1))
        self.P = eye(self.dim_x)
        self.Q = eye(self.dim_x)
        #self.H = zeros((dim_z, dim_x))
        self.R = eye(self.dim_z)
        self.M = zeros((self.dim_z, self.dim_z))

        self._I = eye(self.dim_x)  # This helps the I matrix to always be compatible to the state vector's dim
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)

        self.F = np.array([[1, 0, 1, 0],
                            [0, 1, 0, 1],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Transformation matrix (Observation to State)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        self.R[2:, 2:] *= 10.  # observation error covariance
        self.P[4:, 4:] *= 1000.  # initial velocity error covariance
        self.P *= 10.  # initial location error covariance
        self.Q[-1, -1] *= 0.01  # process noise
        self.Q[4:, 4:] *= 0.01  # process noise

    def predict(self,delta_t=0.):
        '''
        self.F = np.array([[1, 0, 0.5*delta_t, 0],
                            [0, 1, 0, 0.5*delta_t],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        '''
        self.x = dot(self.F, self.x)                                    # x = Fx
        
        #x = dot(self.F, x)
        self.P = dot(self.F, dot(self.P, self.F.T)) + self.Q            # P = FPF' + Q
        self.x_prior = np.copy(self.x)
        #self.x_prior = np.copy(x)
        self.P_prior = np.copy(self.P)
        return self.x
    
    def update(self, z,R=10.):
        if(self.x[0][0]<0.0001):
            self.x[0][0]=0.0001
        px=self.x[0][0]

        

        py=self.x[1][0]
        
        c1=px*px+py*py
        c2=math.sqrt(c1)
        self.H = np.array([[(px/c2), (py/c2), 0, 0],
                           [-(py/c1), (px/c1), 0, 0]])
        #'''
        
        #x_in_polar=[[c2],[math.atan(px/py)]]
        x_in_polar=[[c2],[math.atan(py/px)]]
        #print("z_ ",x_in_polar)
        #print("z  ",z)
        
        y =  np.array(z) - np.array(x_in_polar)
        #print("y  ",y)
        '''
        while(y[[1]]>math.pi):
            y[[1]]=y[[1]]-math.pi
        while(y[[1]]<math.pi):
            y[[1]]=y[[1]]+math.pi
        '''
        #hx= np.dot(self.H, self.x)
        #print("y=z-hx ",round(y[2][0])," = ", z[2][0]," - ", hx[2][0])
        if(R>11):
            y[2][0]=0.
            y[3][0]=0.
        
        PHT = dot(self.P, self.H.T)
        '''
        self.R = np.array([[0.1, 0., 0., 0.],
                           [0., 0.1, 0., 0.],
                           [0., 0., 0.001, 0.],
                           [0., 0., 0., 0.001]])'''
        self.R = np.array([[0.1, 0.1],
                           [0.1, 0.1]])

        # S = HPH' + R (Project system uncertainty into measurement space)
        S = dot(self.H, PHT) + self.R
        #print (S)
        # K = PH'S^-1  (map system uncertainty into Kalman gain)
        K = dot(PHT, inv(S))
        # x = x + Ky  (predict new x with residual scaled by the Kalman gain)
        self.x = self.x + dot(K, y)
        # P = (I-KH)P
        I_KH = self._I - dot(K, self.H)
        self.P = dot(I_KH, self.P)
        #self.kf.update(xxyy_to_xysr(new_detection))
        return self.x


    
