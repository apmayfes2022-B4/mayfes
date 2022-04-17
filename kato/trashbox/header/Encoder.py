import numpy as np

class Encoder():
    def __init__(self):
        self.x = np.zeros(2)
        self.y = np.zeros(2)
        self.z = np.zeros(2)
        self.resolution = 800
        self.trans_coefficient = 1.0/float(self.resolution)
        self.t_diff = 0.01
    
    def set_t_diff(self,time_d):
        self.t_diff = time_d
    
    def Omega(self):
        omega = np.zeros((3,1))
        omega[0] = 2.0*np.pi*self.vel_calc(self.x[1],self.x[0])/self.t_diff*self.trans_coefficient
        omega[1] = 2.0*np.pi*self.vel_calc(self.y[1],self.y[0])/self.t_diff*self.trans_coefficient
        omega[2] = 2.0*np.pi*self.vel_calc(self.z[1],self.z[0])/self.t_diff*self.trans_coefficient
        return omega
    
    def vel_calc(self,after,before):
        velocity = after - before
        if velocity < -self.resolution/4:
            velocity += self.resolution
        elif velocity > self.resolution/4:
            velocity -= self.resolution
        return velocity

        
    def update(self,ee):
        if np.count_nonzero(self.x)+np.count_nonzero(self.y)+np.count_nonzero(self.z)>0:
            self.x[0] = self.x[1]
            self.y[0] = self.y[1]
            self.z[0] = self.z[1]
            self.x[1] = ee['x']
            self.y[1] = ee['y']
            self.z[1] = ee['z']
        else:
            self.x[0] = ee['x']
            self.y[0] = ee['y']
            self.z[0] = ee['z']
            self.x[1] = ee['x']
            self.y[1] = ee['y']
            self.z[1] = ee['z']
        
    def debug(self):
        print('x is ',self.x)
        print('y is ',self.y)
        print('z is ',self.z)
        
    def decode(self,omega):
        self.x[0] += int(omega[0]*self.t_diff/(2.0*np.pi)*self.resolution)
        self.x[0] = np.mod(self.x[0]+self.resolution,self.resolution)
        self.y[0] += int(omega[1]*self.t_diff/(2.0*np.pi)*self.resolution)
        self.y[0] = np.mod(self.y[0]+self.resolution,self.resolution)
        self.z[0] += int(omega[2]*self.t_diff/(2.0*np.pi)*self.resolution)
        self.z[0] = np.mod(self.z[0]+self.resolution,self.resolution)