import numpy as np
from header import EKF
from header import Encoder

def main():
    enc = Encoder.Encoder()
    ekf = EKF.EKF()
    t_diff = 0.01
    enc.set_t_diff(t_diff)
    steps = 1000
    x = np.zeros((3,steps))
    v = np.zeros(3)
    for i in range(steps):
        theta = 2*np.pi*i/steps
        x[2,i] = 0
        x[0,i] = 10*np.cos(theta)-10
        x[1,i] = 10*np.sin(theta)
        if i>0:
            v = (x[:,i]-x[:,i-1])/t_diff
        omega = ekf.obs_evolution(v)
        enc.decode(omega)
        print(0,int(enc.x[0]),int(enc.y[0]),int(enc.z[0]))
        if not i%20:
            print(1,x[0,i],x[1,i])    
    print(2)

if __name__ == "__main__":
    main()