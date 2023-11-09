import numpy as np
from numpy import dot, zeros, eye
from numpy.linalg import inv

class KF:
    def __init__(self):

        delta_t=0
        # Transition matrix
        self.dim_x = 4
        self.dim_z = 4
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
                           [0, 1, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0]])

        self.R[2:, 2:] *= 10.  # observation error covariance
        self.P[4:, 4:] *= 1000.  # initial velocity error covariance
        self.P *= 10.  # initial location error covariance
        self.Q[-1, -1] *= 0.01  # process noise
        self.Q[4:, 4:] *= 0.01  # process noise

    def predict(self,delta_t=0.):
        
        #if self.kf.x[6] + self.kf.x[2] <= 0:
        #    self.kf.x[6] *= 0.0
        #if self.kf.x[7] + self.kf.x[2] <= 0:
        #    self.kf.x[7] *= 0.0
        #self.kf.predict()
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
        """
        Updates track with new observation and returns itself after update
        :param new_detection: (np.ndarray) new observation in format [x1,x2,y1,y2]
        :return: KalmanTrack object class (itself after update)
        """
        '''
        if self.x[1][0]!=0 and self.x[1][0]!=0 :
            self.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [1,0,0,0]])
            #[(640/self.x[0][0])-950/self.x[1][0], -950*self.x[0][0]/(self.x[1][0]*self.x[1][0]), 0, 0]])
            #print("px/py",(640/self.x[0][0])-950*self.x[0][0]/self.x[1][0],-950*self.x[0][0]*self.x[1][0]/(self.x[1][0]*self.x[1][0]))
        else:
            self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 0]])
        '''
        self.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
        #'''
        y = z - np.dot(self.H, self.x)
        hx= np.dot(self.H, self.x)
        #print("y=z-hx ",round(y[2][0])," = ", z[2][0]," - ", hx[2][0])
        if(R>11):
            y[2][0]=0.
            y[3][0]=0.

        PHT = dot(self.P, self.H.T)
        
        self.R = np.array([[0.1, 0., 0., 0.],
                           [0., 0.1, 0., 0.],
                           [0., 0., 0.001, 0.],
                           [0., 0., 0., 0.001]])

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
    
    def update_cam(self, u):
        """
        Updates track with new observation and returns itself after update
        :param new_detection: (np.ndarray) new observation in format [x1,x2,y1,y2]
        :return: KalmanTrack object class (itself after update)
        """
        #R=10.
        if (self.x[1][0]!=0):
            self.H = np.array([[950/self.x[1][0],0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]])
        else:
            self.H = np.array([[0,0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]])
        #'''
        z=[[u],[0],[0],[0]]
        y = z - np.dot(self.H, self.x)
        hx= np.dot(self.H, self.x)
        #my_u=950*self.x[0][0]/self.x[1][0]
        #print("y=z-hx ",round(y[0][0])," = ", z[0][0]," - ", hx[0][0]," my u ",my_u)
        ##if(R>11):
        #    y[2][0]=0.
        #    y[3][0]=0.
        self.R = np.array([[10000., 0., 0., 0.],
                           [0., 10000., 0., 0.],
                           [0., 0., 0.001, 0.],
                           [0., 0., 0., 0.001]])
        PHT = dot(self.P, self.H.T)
        
        #self.R = eye(self.dim_z)
        #self.R[2:, 2:] *= 0.001  # observation error covariance

        # S = HPH' + R (Project system uncertainty into measurement space)
        S = dot(self.H, PHT) + self.R
        # K = PH'S^-1  (map system uncertainty into Kalman gain)
        K = dot(PHT, inv(S))
        # x = x + Ky  (predict new x with residual scaled by the Kalman gain)
        self.x = self.x + dot(K, y)
        # P = (I-KH)P
        I_KH = self._I - dot(K, self.H)
        self.P = dot(I_KH, self.P)
        #self.kf.update(xxyy_to_xysr(new_detection))
        return self.x
    '''
    def updateQ(self, dt):#dt time difference
        dt2 = dt * dt
        dt3 = dt * dt2
        dt4 = dt * dt3
        
        x, y = self.a
        
        r11 = dt4 * x / 4
        r13 = dt3 * x / 2
        r22 = dt4 * y / 4
        r24 = dt3 * y /  2
        r31 = dt3 * x / 2 
        r33 = dt2 * x
        r42 = dt3 * y / 2
        r44 = dt2 * y
        
        Q = np.matrix([[r11, 0, r13, 0],
                    [0, r22, 0, r24],
                    [r31, 0, r33, 0], 
                    [0, r42, 0, r44]])
        
        self.kalmanFilter.setQ(Q)
    '''

    
